% Bundled Camera Path Video Stabilization
% Written by Tan SU
% contact: sutank922@gmail.com

clear all;
addpath('mesh');
addpath('RANSAC');
addpath('mex');

%% Parametres
% -------INPUT-------
inputDir = 'D:/video/fuji/ref/';
outputDir = 'D:/video/fuji/stabres/';
nFrames = 3000;
resize = [720 1280];           % resize the input frame to this size. If input frame size is too large, rendering time might be very slow (MATLAB interp2 is too slow)
% -------TRACK-------
TracksPerFrame = 256;           % number of trajectories in a frame, 200 - 2000 is OK
% -------STABLE------
MeshSize = 8;                   % The mesh size of bundled camera path, 6 - 12 is OK
Smoothness = 1;                 % Adjust how stable the output is, 0.5 - 3 is OK
Span = 30;                      % Omega_t the window span of smoothing camera path, usually set it equal to framerate
Cropping = 1;                   % adjust how similar the result to the original video, usually set to 1
Rigidity = 2;                   % adjust the rigidity of the output mesh, consider set it larger if distortion is too significant, [1 - 4]
iteration = 10;                 % number of iterations when optimizing the camera path [10 - 20]
% -------OUTPUT------
OutputPadding = 200;            % the padding around the video, should be large enough. 

%% Track by KLT
tic;
track = GetTracks(inputDir, resize, MeshSize, TracksPerFrame, nFrames); 
toc;

%% Compute original camera path (by As-similar-as-possible Warping)
tic;
path = getPath(MeshSize, track);    
toc;

%% Optimize the paths
tic;
bundled = Bundled(inputDir, resize, path, Span, Smoothness, Cropping, Rigidity);
bundled.optPath(iteration);
toc;

%% Render the stabilzied frames
tic;
bundled.render(outputDir, OutputPadding);
toc;
