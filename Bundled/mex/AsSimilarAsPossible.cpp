#include "mex.h"
#include <iostream>
#include "../Asap/Asap.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	// check for proper number of input and output arguments
	if(nrhs<6 || nrhs>7)
		mexErrMsgTxt("Only two or three input arguments are allowed!");
	if(nlhs > 1)
		mexErrMsgTxt("Only one output arguments are allowed!");
	
    double alpha = 1;    
	int imageHeight = (int) mxGetScalar(prhs[2]);
    int imageWidth = (int) mxGetScalar(prhs[3]);
    double quadHeight = mxGetScalar(prhs[4]);
    double quadWidth = mxGetScalar(prhs[5]);
	int nDims = mxGetNumberOfDimensions(prhs[0]);
    auto *dims = mxGetDimensions(prhs[0]);
    if (nDims != 2 || dims[1] != 2)
        mexErrMsgTxt("Control Points should be N x 2 array");

    double *cp1 = (double *) mxGetData(prhs[0]);
    double *cp2 = (double *) mxGetData(prhs[1]);

    int ncp = (int) dims[0];
	dims = mxGetDimensions(prhs[1]);
	assert((int)dims[0] == ncp);
	// read into vector of points
	double *p1x, *p1y, *p2x, *p2y;
	p1x = (double *)malloc(ncp * sizeof(double));
	p1y = (double *)malloc(ncp * sizeof(double));
	p2x = (double *)malloc(ncp * sizeof(double));
	p2y = (double *)malloc(ncp * sizeof(double));
	for (int i = 0; i < ncp; ++i)
	{
		p1x[i] = cp1[i];
		p1y[i] = cp1[i + ncp];
		p2x[i] = cp2[i];
		p2y[i] = cp2[i + ncp];
	}

    if (nrhs==7)
    {
        alpha = mxGetScalar(prhs[6]);
    }
        
    //mexPrintf("Image Size %dx%d\tQuad Size: %.2fx%.2f\talpha: %f\t# of Control Points: %d", imageHeight, imageWidth, quadHeight, quadWidth, alpha, ncp);
	//mexPrintf("alpha: %f   ratio: %f   minWidth: %d  nOuterFPIterations: %d  nInnerFPIterations: %d   nCGIterations: %d\n",alpha,ratio,minWidth,nOuterFPIterations,nInnerFPIterations,nCGIterations);

	Asap::Asap asap(imageHeight, imageWidth, quadHeight, quadWidth, alpha);
	asap.addControlPoints(ncp, p1x, p1y, p2x, p2y);	
	asap.solve();
	//testComputeHomography();
	vector<vector<Asap::Homography>> homos = asap.calcHomos();
	auto meshHeight = homos.size();
	auto meshWidth = homos[0].size();
	mwSize outDims[4] = { meshHeight, meshWidth, 3, 3 };
	plhs[0] = mxCreateNumericArray(4, outDims, mxDOUBLE_CLASS, mxREAL);
	auto pdata = mxGetPr(plhs[0]);
	for (int row = 0; row < meshHeight; ++row)
	{
		for (int col = 0; col < meshWidth; ++col)
		{
			auto H = homos[row][col];			
			for (int i = 0; i < 3; ++i)
				for (int j = 0; j < 3; ++j)
				{
					auto index = j * 3 * meshWidth * meshHeight + i * meshWidth * meshHeight + col * meshHeight + row;
					pdata[index] = H.getMat()(i, j);
				}
		}
	}
}
