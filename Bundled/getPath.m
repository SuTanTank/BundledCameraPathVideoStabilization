function path = getPath(MeshSize, tracks)
%GETPATH Compute the bundled camera path
%   track is the set of trajectories of the input video, backList indicate
%   the background trajectories.     
    nFrames = tracks.nFrame;
    if nFrames < 2
        error('Wrong inputs') ;
    end
    path = zeros(nFrames, MeshSize, MeshSize, 3, 3);
    for row = 1:MeshSize
        for col = 1:MeshSize
            path(1, row, col, :, :) = eye(3);
        end
    end
    fprintf('%5d', 1);
    for frameIndex = 2:nFrames
        fprintf('%5d', frameIndex);
        if mod(frameIndex, 20) == 0
            fprintf('\n') ;
        end                
        quadH = tracks.videoHeight / MeshSize;
        quadW = tracks.videoWidth / MeshSize;
        asaplambda = 3;
        [I1_features,I2_features] = tracks.getF(frameIndex - 1);
        homos = NewWarping(I1_features, I2_features, tracks.videoHeight, tracks.videoWidth, quadH, quadW, asaplambda);
        for i = 1:MeshSize
            for j = 1:MeshSize
                path(frameIndex, i, j, :, :) = squeeze(homos(i, j, :, :)) * squeeze(path(frameIndex - 1, i, j, :, :));
                path(frameIndex, i, j, :, :) = squeeze(path(frameIndex, i, j, :, :)) / path(frameIndex, i, j, 3, 3);
            end
        end            
    end
end

