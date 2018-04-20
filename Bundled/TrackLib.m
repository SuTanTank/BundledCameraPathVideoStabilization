classdef TrackLib < handle    
    properties
        videoWidth;
        videoHeight;
        nTrack;
        list;
        wSize;
        head;
        tail;
        id;
        len;
        points;
        labels;
        nFrame;
        live;
        nLabel;
        nWindow;
        labelMask;
    end
    
    methods
        function obj = TrackLib(nFrame)
            obj.points = [];
            obj.nFrame = nFrame;
            obj.nTrack = 0;
            obj.live = [];
            obj.head = [];
            obj.len = [];
            obj.tail = [];
        end   
        
        function addPoints(obj, newPoints, frameIndex)
            nNew = size(newPoints, 1);
            obj.points = [obj.points; zeros(nNew, obj.nFrame, 2)];            
            obj.points(obj.nTrack + 1:obj.nTrack + nNew, frameIndex, :) = reshape(newPoints, [nNew, 1, 2]);            
            obj.live = [obj.live obj.nTrack + 1: obj.nTrack + nNew];            
            obj.head = [obj.head; ones(nNew, 1) * frameIndex];
            obj.tail = [obj.tail; ones(nNew, 1) * frameIndex];
            obj.nTrack = obj.nTrack + nNew;
            obj.len = [obj.len; ones(nNew, 1)];
        end
        
        function updatePoints(obj, uPoints, frameIndex)
            obj.points(obj.live, frameIndex, :)  = reshape(uPoints, [length(obj.live), 1, 2]);
            obj.len(obj.live) = obj.len(obj.live) + 1;
        end
        
        function endPoints(obj, validity, frameIndex)
            obj.tail(obj.live(validity == false)) = frameIndex - 1;            
            obj.live(validity == false) = [];
        end
        
        function [M2, id] = getM(obj, index)
            left = (index - 1) * obj.wSize / 2 + 1;
            right = left + obj.wSize - 1;
            mid = (left + right) / 2.0;
            sub = obj.points(:, left:right, :) ;
            valid = 1:obj.nTrack;
            valid = valid(obj.head < mid & obj.tail > mid & obj.len > obj.wSize * 0.5);
            sub = sub(valid, :, :);
            id = valid;
            M2 = permute(sub, [3 1 2]);
        end
        
        function [f1, f2] = getF(obj, frameIndex)
            selected = obj.head <= frameIndex & obj.tail > frameIndex & obj.len > 4;
            f1 = squeeze(obj.points(selected, frameIndex, :));
            f2 = squeeze(obj.points(selected, frameIndex + 1, :));
            [~,inliners] = EstimateHomographyByRANSAC(f1',f2', 0.005);
            f1 = f1(inliners,:);
            f2 = f2(inliners,:);
        end
        function [f1, f2] = getAllF(obj, frameIndex)
            selected = obj.head <= frameIndex & obj.tail > frameIndex;
            f1 = squeeze(obj.points(selected, frameIndex, :));
            f2 = squeeze(obj.points(selected, frameIndex + 1, :));           
        end
    end
    
end

