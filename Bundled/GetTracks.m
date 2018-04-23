function tracks = GetTracks( input, meshSize, demand, nFrames)
%GetTracks Compute tracks by KLT
%   Use KLT to track evenly fistributed track points
%   input: the path to images
%   meshSize: the meshSize of video stitching
    fileList = dir(input);
    fileList = fileList(3:length(fileList));
    if ~exist('nFrames', 'var') || nFrames > length(fileList)
        nFrames = length(fileList); 
    end
    
    tracks = TrackLib(nFrames);    
    tracks.maxLen = 300;
    
    
    tracker = vision.PointTracker('MaxBidirectionalError', 1);
    fileName = fileList(1).name;
    frame = imread([input fileName]);
    [H, W, ~] = size(frame);    
    tracks.videoWidth = W;    
    tracks.videoHeight = H;
    
    tracks.scale = 540 / H;
    frame_s = imresize(frame, tracks.scale);
    
    [H, W, ~] = size(frame);    
    tracks.videoWidth = W;    
    tracks.videoHeight = H;
    livePoints = getMorePoints(frame_s, meshSize, 0, [], demand);
    initialize(tracker, livePoints, frame_s);
    tracks.addPoints(livePoints, 1);
    for frameIndex = 2:nFrames
        fprintf('%5d', frameIndex);
        if mod(frameIndex, 20) == 0
            fprintf('\n') ;
        end        
        fileName = fileList(frameIndex).name;
        frame = imread([input fileName]);
        frame_s = imresize(frame, tracks.scale);
        
        
        [livePoints, validity] = step(tracker, frame_s);
        age = true(size(validity));
        age(tracks.len(tracks.live) == tracks.maxLen) = false;
%         fprintf('=> %d\t%d\n', size(tracks.live, 2), size(validity, 1));
        if size(tracks.live, 2) ~= size(validity, 1)
            disp('?') ;
        end
        tracks.endPoints(validity & age, frameIndex);
        tracks.updatePoints(livePoints(validity & age, :), frameIndex);
        
        % end too old tracks 
        morePoints = getMorePoints(frame_s, meshSize, length(tracks.live), livePoints(validity == true, :), demand);
        tracks.addPoints(morePoints, frameIndex);
        livePoints = [livePoints(validity & age, :); morePoints];
        setPoints(tracker, livePoints);
        marked = insertMarker(frame_s, livePoints);
        imshow(marked);        
    end
    tracks.endPoints(false(length(tracks.live), 1), length(fileList) + 1);
end

function pointsMore = getMorePoints(frame, meshSize, nP, oldpoints, demand)
    demand = demand / (meshSize * meshSize);
    votes = zeros(meshSize);
    [H, W, ~] = size(frame);
    threshold = 0.5;
    if nP > 0
        votes = getVotes(frame, meshSize, oldpoints);
    end
    points = [];
    
    for row = 1:meshSize
        for col = 1:meshSize
            if votes(row, col) < demand * 0.8
                nMore = floor(demand - votes(row, col));
                roi = [1 + (col - 1) * W / meshSize, 1 + (row - 1) * H / meshSize, W / meshSize - 1, H / meshSize - 1];                
                pNew = detectMinEigenFeatures(rgb2gray(frame), 'ROI', roi, 'MinQuality', threshold); 
                while (size(pNew, 1) < nMore) && threshold > 0.1
                    threshold = threshold - 0.1; 
                    threshold = max(threshold, 0);
                    pNew = detectMinEigenFeatures(rgb2gray(frame), 'ROI', roi, 'MinQuality', threshold); 
                end
                if nMore < size(pNew, 1)
                    pNew = pNew.selectStrongest(nMore);
                end
                points = [points; pNew.Location];
            end
        end
    end
    pointsMore = points;
    
end

function votes = getVotes(frame, meshSize, points)
    [H, W, ~] = size(frame);    
    qH = H / meshSize;
    qW = W / meshSize;
    index = floor(points(:, 1) / qW) * meshSize + (floor(points(:, 2) / qH)) + 1; 
    voting = histcounts([index; 1; meshSize*meshSize], meshSize*meshSize);  
    voting(1) = voting(1) - 1;
    voting(meshSize * meshSize) = voting(meshSize*meshSize) - 1;    
    votes = reshape(voting, [meshSize meshSize]);
end
