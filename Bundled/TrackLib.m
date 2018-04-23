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
        scale;
        maxLen;
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
            obj.maxLen = 200;
            obj.scale = 1;
        end   
        
        function addPoints(obj, newPoints, frameIndex)
            nNew = size(newPoints, 1);
            if (size(obj.points, 1) < obj.nTrack + nNew)
                obj.points = [obj.points; zeros(50000, obj.nFrame, 2)];            
            end           
            obj.points(obj.nTrack + 1:obj.nTrack + nNew, frameIndex, :) = reshape(newPoints, [nNew, 1, 2]) / obj.scale;            
            obj.live = [obj.live obj.nTrack + 1: obj.nTrack + nNew];            
            obj.head = [obj.head; ones(nNew, 1) * frameIndex];
            obj.tail = [obj.tail; ones(nNew, 1) * frameIndex];
            obj.nTrack = obj.nTrack + nNew;
            obj.len = [obj.len; ones(nNew, 1)];
        end
        
        function updatePoints(obj, uPoints, frameIndex)
            obj.points(obj.live, frameIndex, :)  = reshape(uPoints, [length(obj.live), 1, 2]) / obj.scale;
            obj.len(obj.live) = obj.len(obj.live) + 1;
        end
        
        function endPoints(obj, validity, frameIndex)
            obj.tail(obj.live(validity == false)) = frameIndex - 1;            
            obj.live(validity == false) = [];
        end
        
        function [f1, f2] = getF(obj, frameIndex)
            selected = obj.head <= frameIndex & obj.tail > frameIndex & obj.len > 20;
            f1 = squeeze(obj.points(selected, frameIndex, :));
            f2 = squeeze(obj.points(selected, frameIndex + 1, :));
            [~,inliners] = ransacfithomography(f1',f2', 0.002);
            f1 = f1(inliners,:);
            f2 = f2(inliners,:);
        end
        function [f1, f2] = getAllF(obj, frameIndex)
            selected = obj.head <= frameIndex & obj.tail > frameIndex & obj.len > 20;
            f1 = squeeze(obj.points(selected, frameIndex, :));
            f2 = squeeze(obj.points(selected, frameIndex + 1, :));           
        end
    end
    
end

