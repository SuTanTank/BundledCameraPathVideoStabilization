# Matlab Implementation of Bundled Camera Path Video Stabilization

This is a 3rd party implementation of the paper ___Bundled Camera Path Video Stabilization___ [SIGGRAPH 2013]] The original paper's author is Liu Shuaicheng. (Liu's personal website: http://www.liushuaicheng.org)

*Some source code are also provided by Liu Shuaicheng, the original copy can be downloaded from this webpage* http://www.liushuaicheng.org/SIGGRAPH2013/index.htm

## Usage:
1. Extract frames of your video to a folder. The file names should be indexed properly. (e.g. ***001.png 002.png*** ...) `ffmpeg -i Input-Video-File-Name.mp4 Input_Frame_Path/%04d.png`
2. Set the input and output path in `run.m`
3. Set up the parameters, see comments for more detail. 
4. run `run.m` 