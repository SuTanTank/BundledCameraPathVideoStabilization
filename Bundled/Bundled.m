classdef Bundled < handle    
    properties
        % inputs
        seq;
        % Dimensions
        nFrames;        
        videoHeight;
        videoWidth;        
        
        meshSize;        
        quadHeight;
        quadWidth;
        
        % optimization parameters
        span;
        smoothness;
        cropping;        
        stableW;
        gap;
        
        % Optimization data
        P;        
        C;
        
        w;        
        gamma;
       
    end
    
    methods
        function obj = Bundled(seq, path, span, smoothness, cropping, rigidity)
            obj.seq = seq;
            obj.C = path;            
            obj.P = obj.C;            
            obj.smoothness = smoothness;
            obj.cropping = cropping;            
            obj.stableW = 20 * rigidity;
            obj.span = span;
            
			fileList = dir(seq);
			fileList = fileList(3:length(fileList));
			obj.nFrames = size(path, 1);
			if obj.nFrames < 2
				error('Wrong inputs directory') ;
			end
			frame = imread([seq fileList(1).name]);
		
            [obj.videoHeight, obj.videoWidth, ~] = size(frame);
            [~, obj.meshSize, ~, ~, ~] = size(path);
            
            obj.quadHeight = obj.videoHeight / obj.meshSize;
            obj.quadWidth = obj.videoWidth / obj.meshSize;
            
            obj.w = zeros(obj.nFrames, 2 * obj.span + 1, obj.meshSize, obj.meshSize);
            
            obj.gamma = zeros(obj.nFrames, obj.meshSize, obj.meshSize);            
        end
        
        function calcOmega(obj)
            for i = 1:obj.meshSize
                for j = 1:obj.meshSize
                    for t = 1:obj.nFrames                        
                        for r = t-obj.span:t+obj.span
%                             if t > obj.span && t <= obj.nFrames - obj.span
                                if r > 0 && r <= obj.nFrames
                                    r_offset = r - t + obj.span + 1;
                                    dP = abs(obj.C(t,i,j,1,3) - obj.C(r,i,j,1,3)) + abs(obj.C(t,i,j,2,3) - obj.C(r,i,j,2,3));                                
                                    obj.w(t,r_offset,i,j) = gaussmf(abs(t-r), [10 0]) * gaussmf(dP, [800 0]);
                                    obj.w(t,obj.span + 1,i,j) = 0;        
                                end
%                             end
                        end
                        obj.gamma(t,i,j) = sum(obj.w(t,:,i,j)) * 2 * obj.smoothness;                        
                        obj.gamma(t,i,j) = obj.gamma(t,i,j) + 1;                        
                        if (1 == obj.meshSize) || obj.stableW == 0
                            continue;
                        end
                        if ((i==1)||(i==obj.meshSize))&&((j==1)||(j==obj.meshSize))
                            obj.gamma(t,i,j) = obj.gamma(t,i,j) + 2 * 3 * obj.stableW;                            
                        end
                        if ((i==1)||(i==obj.meshSize))&&((j>1)&&(j<obj.meshSize))
                            obj.gamma(t,i,j) = obj.gamma(t,i,j) + 2 * 5 * obj.stableW;                            
                        end
                        if ((i>1)&&(i<obj.meshSize))&&((j==1)||(j==obj.meshSize))
                            obj.gamma(t,i,j) = obj.gamma(t,i,j) + 2 * 5 * obj.stableW;                            
                        end
                        if ((i>1)&&(i<obj.meshSize))&&((j>1)&&(j<obj.meshSize))
                            obj.gamma(t,i,j) = obj.gamma(t,i,j) + 2 * 8 * obj.stableW;                            
                        end
                    end
                end
            end            
        end
               
        function value = getCoherenceTerm(obj, C, P, row, col, frameIndex)
            value = 0;
            if obj.meshSize == 1
                return 
            end
            % i=1, j=1
            if row == 1 && col == 1
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row+1, col, :, :)) / squeeze(C(frameIndex, row+1, col, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row, col+1, :, :)) / squeeze(C(frameIndex, row, col+1, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row+1, col+1, :, :)) / squeeze(C(frameIndex, row+1, col+1, :, :)) * squeeze(C(frameIndex, row, col, :, :));
            end
            % row=1, col=2:15
            if row == 1 && col < obj.meshSize && col > 1
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row+1, col, :, :)) / squeeze(C(frameIndex, row+1, col, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row, col+1, :, :)) / squeeze(C(frameIndex, row, col+1, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row+1, col+1, :, :)) / squeeze(C(frameIndex, row+1, col+1, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row+1, col-1, :, :)) / squeeze(C(frameIndex, row+1, col-1, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row, col-1, :, :)) / squeeze(C(frameIndex, row, col-1, :, :)) * squeeze(C(frameIndex, row, col, :, :));
            end
            % row=1, col=16
            if row == 1 && col == obj.meshSize
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row+1, col, :, :)) / squeeze(C(frameIndex, row+1, col, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row, col-1, :, :)) / squeeze(C(frameIndex, row, col-1, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row+1, col-1, :, :)) / squeeze(C(frameIndex, row+1, col-1, :, :)) * squeeze(C(frameIndex, row, col, :, :));
            end
            % row=2:15, col=1
            if col == 1 && row < obj.meshSize && row > 1
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row+1, col, :, :)) / squeeze(C(frameIndex, row+1, col, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row-1, col, :, :)) / squeeze(C(frameIndex, row-1, col, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row+1, col+1, :, :)) / squeeze(C(frameIndex, row+1, col+1, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row-1, col+1, :, :)) / squeeze(C(frameIndex, row-1, col+1, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row, col+1, :, :)) / squeeze(C(frameIndex, row, col+1, :, :)) * squeeze(C(frameIndex, row, col, :, :));
            end
            % row=2:15, col=2:15
            if row > 1 && row < obj.meshSize && col > 1 && col < obj.meshSize
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row+1, col, :, :)) / squeeze(C(frameIndex, row+1, col, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row-1, col, :, :)) / squeeze(C(frameIndex, row-1, col, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row+1, col+1, :, :)) / squeeze(C(frameIndex, row+1, col+1, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row-1, col+1, :, :)) / squeeze(C(frameIndex, row-1, col+1, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row-1, col-1, :, :)) / squeeze(C(frameIndex, row-1, col-1, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row+1, col-1, :, :)) / squeeze(C(frameIndex, row+1, col-1, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row, col+1, :, :)) / squeeze(C(frameIndex, row, col+1, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row, col-1, :, :)) / squeeze(C(frameIndex, row, col-1, :, :)) * squeeze(C(frameIndex, row, col, :, :)); 
            end
            % row=2:15, col=16
            if col == obj.meshSize && row < obj.meshSize && row > 1
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row+1, col, :, :)) / squeeze(C(frameIndex, row+1, col, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row, col-1, :, :)) / squeeze(C(frameIndex, row, col-1, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row-1, col, :, :)) / squeeze(C(frameIndex, row-1, col, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row+1, col-1, :, :)) / squeeze(C(frameIndex, row+1, col-1, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row-1, col-1, :, :)) / squeeze(C(frameIndex, row-1, col-1, :, :)) * squeeze(C(frameIndex, row, col, :, :));
            end
            % row=16, col=1
            if row == obj.meshSize && col == 1
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row-1, col, :, :)) / squeeze(C(frameIndex, row-1, col, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row, col+1, :, :)) / squeeze(C(frameIndex, row, col+1, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row-1, col+1, :, :)) / squeeze(C(frameIndex, row-1, col+1, :, :)) * squeeze(C(frameIndex, row, col, :, :));
            end
            % row=16, col=2:15
            if row == obj.meshSize && col < obj.meshSize && col > 1
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row-1, col, :, :)) / squeeze(C(frameIndex, row-1, col, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row, col+1, :, :)) / squeeze(C(frameIndex, row, col+1, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row-1, col+1, :, :)) / squeeze(C(frameIndex, row-1, col+1, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row-1, col-1, :, :)) / squeeze(C(frameIndex, row-1, col-1, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row, col-1, :, :)) / squeeze(C(frameIndex, row, col-1, :, :)) * squeeze(C(frameIndex, row, col, :, :));
            end
            % row=16, col=16
            if row == obj.meshSize && col == obj.meshSize
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row-1, col, :, :)) / squeeze(C(frameIndex, row-1, col, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row, col-1, :, :)) / squeeze(C(frameIndex, row, col-1, :, :)) * squeeze(C(frameIndex, row, col, :, :));
                value = value + 2 * obj.stableW * squeeze(P(frameIndex, row-1, col-1, :, :)) / squeeze(C(frameIndex, row-1, col-1, :, :)) * squeeze(C(frameIndex, row, col, :, :));
            end
            
        end
        
        function optPath(obj, maxIte)                        
            obj.P = obj.C;
            obj.calcOmega();            
            for inIte = 1:maxIte
                fprintf('.');
                oP = obj.P;                
                for frameIndex = 1:obj.nFrames
                    for row = 1:obj.meshSize
                        for col = 1:obj.meshSize
                            head = max(frameIndex - obj.span, 1);
                            tail = min(frameIndex + obj.span, obj.nFrames);
                            nn = tail - head + 1;
                            % Pat - Cat
                            value = squeeze(oP(frameIndex, row, col, :, :));                            
                            % Pat - Par
                            n9 = reshape(oP(head:tail, row, col, :, :), [nn, 9]);
                            weight = obj.w(frameIndex, head - frameIndex + obj.span + 1:tail - frameIndex + obj.span + 1, row, col);
                            value = value + 2 * obj.smoothness * reshape(weight * n9, [3, 3]);
                            % Pat - Pat
                            value = value + obj.getCoherenceTerm(obj.C, oP, row, col, frameIndex);
                            % Pat - Pbt'                                
                            obj.P(frameIndex, row, col, :, :) = value / obj.gamma(frameIndex, row, col);
                        end
                    end
                end
            end 
        end
        
        function render(obj, outPath, gap)
            if ~exist(outPath, 'dir')
                mkdir(outPath);
            end
            obj.gap = gap;
            fileList = dir(obj.seq);
            fileList = fileList(3:length(fileList));                
            for frameIndex = 1 : obj.nFrames %parfor
                disp(['rendering: # ' int2str(frameIndex)]);
                fileName = fileList(frameIndex).name;                                
                I = imread([obj.seq fileName]);                
                warp = obj.render1(I, frameIndex, obj.P, obj.C);                
                imwrite(uint8(warp), [outPath '/' int2str(frameIndex) '.bmp']);                
            end
        end
        
        function imwarp = render1(obj, I, frameIndex, P, C)
            src = Mesh(obj.videoHeight, obj.videoWidth, obj.quadWidth, obj.quadHeight);
            des = Mesh(obj.videoHeight, obj.videoWidth, obj.quadWidth, obj.quadHeight);
            
            for i = 0 : obj.meshSize
                for j = 0 : obj.meshSize
                    x = i * obj.quadHeight + 1;
                    y = j * obj.quadWidth + 1;
                    if i == 0 && j == 0
                        B11 = squeeze(P(frameIndex, i+1, j+1, :, :)) / squeeze(C(frameIndex, i+1, j+1, :, :));
                        [xx11, yy11] = obj.transform([y x], B11);
                        des.setVertex(i,j, myPoint(xx11, yy11));
                        continue;
                    end
                    if i == 0 && j == obj.meshSize
                        B10 = squeeze(P(frameIndex, i+1, j, :, :)) / squeeze(C(frameIndex, i+1, j, :, :)) ;
                        [xx10, yy10] = obj.transform([y x], B10);
                        des.setVertex(i,j, myPoint(xx10, yy10));
                        continue;
                    end
                    if i == 0 && j > 0 && j < obj.meshSize
                        B11 = squeeze(P(frameIndex, i+1, j+1, :, :)) / squeeze(C(frameIndex, i+1, j+1, :, :));
                        B10 = squeeze(P(frameIndex, i+1, j, :, :)) / squeeze(C(frameIndex, i+1, j, :, :));
                        [xx11, yy11] = obj.transform([y x], B11);
                        [xx10, yy10] = obj.transform([y x], B10);
                        des.setVertex(i,j, myPoint(mean([xx10 xx11]), mean([yy10 yy11])));
                        continue;
                    end
                    if i>0 && i < obj.meshSize && j == 0
                        B11 = squeeze(P(frameIndex, i+1, j+1, :, :)) / squeeze(C(frameIndex, i+1, j+1, :, :));
                        B01 = squeeze(P(frameIndex, i, j+1, :, :)) / squeeze(C(frameIndex, i, j+1, :, :));
                        [xx11, yy11] = obj.transform([y x], B11);
                        [xx01, yy01] = obj.transform([y x], B01);
                        des.setVertex(i,j, myPoint(mean([xx01 xx11]), mean([yy01 yy11])));
                        continue;
                    end
                    if i > 0 && i < obj.meshSize && j > 0 && j < obj.meshSize
                        B11 = squeeze(P(frameIndex, i+1, j+1, :, :)) / squeeze(C(frameIndex, i+1, j+1, :, :));
                        B01 = squeeze(P(frameIndex, i, j+1, :, :)) / squeeze(C(frameIndex, i, j+1, :, :));
                        B00 = squeeze(P(frameIndex, i, j, :, :)) / squeeze(C(frameIndex, i, j, :, :));
                        B10 = squeeze(P(frameIndex, i+1, j, :, :)) / squeeze(C(frameIndex, i+1, j, :, :));
                        [xx11, yy11] = obj.transform([y x], B11);
                        [xx01, yy01] = obj.transform([y x], B01);
                        [xx10, yy10] = obj.transform([y x], B10);
                        [xx00, yy00] = obj.transform([y x], B00);
                        [xx, yy] = obj.mergepoints([xx01, xx11, xx10, xx00], [yy01, yy11, yy10, yy00]);
                        des.setVertex(i,j, myPoint(xx, yy));
                        continue;
                    end
                    if i>0 && i < obj.meshSize && j == obj.meshSize
                        B00 = squeeze(P(frameIndex, i, j, :, :)) / squeeze(C(frameIndex, i, j, :, :));
                        B10 = squeeze(P(frameIndex, i+1, j, :, :)) / squeeze(C(frameIndex, i+1, j, :, :));
                        [xx10, yy10] = obj.transform([y x], B10);
                        [xx00, yy00] = obj.transform([y x], B00);
                        des.setVertex(i,j, myPoint(mean([xx10 xx00]), mean([yy10 yy00])));
                        continue;
                    end
                    if i == obj.meshSize && j == 0
                        B01 = squeeze(P(frameIndex, i, j+1, :, :)) / squeeze(C(frameIndex, i, j+1, :, :));
                        [xx01, yy01] = obj.transform([y x], B01);
                        des.setVertex(i,j, myPoint(xx01, yy01));
                        continue;
                    end
                    if i == obj.meshSize && j>0 && j < obj.meshSize
                        B00 = squeeze(P(frameIndex, i, j, :, :)) / squeeze(C(frameIndex, i, j, :, :));
                        B01 = squeeze(P(frameIndex, i, j+1, :, :)) / squeeze(C(frameIndex, i, j+1, :, :));
                        [xx00, yy00] = obj.transform([y x], B00);
                        [xx01, yy01] = obj.transform([y x], B01);
                        des.setVertex(i,j, myPoint(mean([xx01 xx00]), mean([yy01 yy00])));
                        continue;
                    end
                    if i == obj.meshSize && j == obj.meshSize
                        B00 = squeeze(P(frameIndex, i, j, :, :)) / squeeze(C(frameIndex, i, j, :, :));
                        [xx00, yy00] = obj.transform([y x], B00); 
                        des.setVertex(i,j, myPoint(xx00, yy00));
                        continue;
                    end
                end
            end
            imwarp = zeros(obj.videoHeight+obj.gap*2,obj.videoWidth+obj.gap*2,3);

            for i=1:obj.meshSize
                for j=1:obj.meshSize
                    p0 = src.getVertex(i-1,j-1);
                    p1 = src.getVertex(i-1,j);
                    p2 = src.getVertex(i,j-1);
                    p3 = src.getVertex(i,j);

                    q0 = des.getVertex(i-1,j-1);
                    q1 = des.getVertex(i-1,j);
                    q2 = des.getVertex(i,j-1);
                    q3 = des.getVertex(i,j);

                    qd1 = Quad(p0,p1,p2,p3);
                    qd2 = Quad(q0,q1,q2,q3);
                    imwarp = quadWarp(obj,I,qd1,qd2, imwarp);
                end
            end 
        end
        
        function [xx, yy] = mergepoints(~, x, y)
            d = ones(4, 4) * 1e8;
            for i = 1:3
                for j = i+1:4
                    d(i, j) = (x(i) - x(j))^2 + (y(i) - y(j))^2;
                end
            end
            [minrow, min_i] = min(d);
            [~, min_j] = min(minrow);
            xx = 0.5 * (x(min_i(min_j))+ x(min_j));
            yy = 0.5 * (y(min_i(min_j))+ y(min_j));            
        end
        
        function imwarp = quadWarp(obj,im,q1,q2, imwarp)
            
            minx = q2.getMinX();
            maxx = q2.getMaxX();
            miny = q2.getMinY();
            maxy = q2.getMaxY();
            
            source = zeros(4,2);
            target = zeros(4,2);
            
            source(1,1) = q2.V00.x;source(1,2) = q2.V00.y;
            source(2,1) = q2.V01.x;source(2,2) = q2.V01.y;
            source(3,1) = q2.V10.x;source(3,2) = q2.V10.y;
            source(4,1) = q2.V11.x;source(4,2) = q2.V11.y;
            
            target(1,1) = q1.V00.x;target(1,2) = q1.V00.y;
            target(2,1) = q1.V01.x;target(2,2) = q1.V01.y;
            target(3,1) = q1.V10.x;target(3,2) = q1.V10.y;
            target(4,1) = q1.V11.x;target(4,2) = q1.V11.y;
            
            HH = homography_4pts(source',target');
            HH = HH./HH(3,3);

            imwarp = myWarp(minx,maxx,miny,maxy,double(im),imwarp,HH,obj.gap);
            imwarp = uint8(imwarp);            
        end
        
        function [x,y] = transform(~, xxyy, B)
            xx = xxyy(1); yy = xxyy(2);
            res = B * [xx;yy;1];
            x = res(1)/res(3);
            y = res(2)/res(3);
        end
        
    end
end

