% Bundled Camera Path Video Stabilization
% Written by Tan SU
% contact: sutank922@gmail.com
addpath('mesh');
addpath('RANSAC');
addpath('mex');

%% params
dataDir = '../stable_data/e2/temp/';
inputDir = '../stable_data/e2/frames/';
outputDir = '../stable_data/e2/result/';
% ---------------
TracksPerFrame = 2000; % number of trajectories in a frame, 200 - 2000 is OK
% ---------------
MeshSize = 12; % The mesh size of bundled camera path, 6 - 12 is OK
Smoothness = 1; % adjust how stable the output is, 0.5 - 3 is OK
Cropping = 1; % adjust how similar the result to the original video, usually set to 1
iteration = 15; % number of iterations when optimizing the camera path
% ---------------
OutputPadding = 200; % the padding around the video, should be large enough. 

%% Track by KLT
tic;
if ~exist([dataDir 'tracks' int2str(TracksPerFrame) '.mat'], 'file')
    track = GetTracks(inputDir, 10, TracksPerFrame); % 10 = tracks evenly distributed in 10*10 grids
    save([dataDir 'tracks' int2str(TracksPerFrame) '.mat'], 'track');
else
    load([dataDir 'tracks' int2str(TracksPerFrame) '.mat']);
end
toc;
close all;

%% Compute original camera path (by As-similar-as-possible Warping)
% the rigidity can be controlled by setting asaplambda inside getPath.m
tic;
if ~exist([dataDir 'Path' int2str(MeshSize) '.mat'], 'file')
    path = getPath(MeshSize, track);    
    save([dataDir 'Path' int2str(MeshSize) '.mat'], 'path');
else
    load([dataDir 'Path' int2str(MeshSize) '.mat']);
end
toc;

%% Optimize the paths
tic;
bundled = Bundled(inputDir, path, Smoothness, Cropping);
bundled.optPath(iteration);
bundled.render(outputDir, OutputPadding);
toc;