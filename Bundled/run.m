% Bundled Camera Path Video Stabilization
% Written by Tan SU
% contact: sutank922@gmail.com

clear all;
addpath('mesh');
addpath('RANSAC');
addpath('mex');

%% params
inputDir = '../stable_data/e2/frames/';
outputDir = '../stable_data/e2/result/';
nFrames = 6000;
% ---------------
TracksPerFrame = 512; % number of trajectories in a frame, 200 - 2000 is OK
% ---------------
MeshSize = 8; % The mesh size of bundled camera path, 6 - 12 is OK
Smoothness = 2; % adjust how stable the output is, 0.5 - 3 is OK
Span = 60; % Omega_t the window span of smoothing camera path
Cropping = 1; % adjust how similar the result to the original video, usually set to 1
iteration = 20; % number of iterations when optimizing the camera path
% ---------------
OutputPadding = 200; % the padding around the video, should be large enough. 

%% Track by KLT
tic;
track = GetTracks(inputDir, MeshSize, TracksPerFrame, nFrames); 
toc;

%% Compute original camera path (by As-similar-as-possible Warping)
tic;
path = getPath(MeshSize, track);    
toc;

%% Optimize the paths
tic;
bundled = Bundled(inputDir, path, Smoothness, Cropping);
bundled.span = Span;
bundled.optPath(iteration);
toc;

%% Render the stabilzied frames
tic;
bundled.render(outputDir, OutputPadding);
toc;
