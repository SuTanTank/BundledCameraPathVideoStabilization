% Bundled Camera Path Video Stabilization
% Written by Tan SU
% contact: sutank922@gmail.com
addpath('mesh');
addpath('RANSAC');
addpath('stitch');
addpath('peter');
addpath('mex');
%% params
data = '../stable_data/';
input = 'frames/';
% ---------------
TracksPerFrame = 2000; % # of trajectories in a frame
% ---------------
MeshSize = 8; % The mesh size of bundled camera path, 5 - 10 is OK
Smoothness = 1; % adjust how stable the output is, 0.5 - 3 is OK
Cropping = 1; % adjust how similar the result to the original video, usually set to 1;
% ---------------
OutputPadding = 200; % the padding around the video
OutputPath = 'res_demo'; % the directory to store the output frames, auto create it if not exist

%% Track by KLT
tic;
if ~exist([data 'tracks' int2str(TracksPerFrame) '.mat'], 'file')
    track = GetTracks([data input], 10, TracksPerFrame); % 10 = tracks evenly distributed in 10*10 grids
    save([data 'tracks' int2str(TracksPerFrame) '.mat'], 'track');
else
    load([data 'tracks' int2str(TracksPerFrame) '.mat']);
end
toc;
close all;
%% Compute original camera path (by As-similar-as-possible Warping)
% the rigidity can be controlled by setting asaplambda inside getPath.m
tic;
if ~exist([data 'Path' int2str(MeshSize) '.mat'], 'file')
    path = getPath(MeshSize, track);    
    save([data 'Path' int2str(MeshSize) '.mat'], 'path');
else
    load([data 'Path' int2str(MeshSize) '.mat']);
end
toc;

%% Optimize the paths
tic;
bundled = Bundled([data input], path, Smoothness, Cropping);
bundled.optPath(20);

bundled.render([data OutputPath], OutputPadding);
toc;