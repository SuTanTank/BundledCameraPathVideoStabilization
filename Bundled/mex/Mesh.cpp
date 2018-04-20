#include "Mesh.h"

Mesh::Mesh(int rows, int cols, double qH, double qW)
{
	imgCols = cols;
	imgRows = rows;
	quadHeight = qH;
	quadWidth = qW;

	vector<double> xSet, ySet;
	double x = 0;
	xSet.push_back(x);
	while (imgCols - x > 0.5 * quadWidth)
	{		
		x += quadWidth;
		xSet.push_back(x);
	}
	double y = 0;
	ySet.push_back(y);
	while (imgRows - y > 0.5 * quadHeight)
	{
		y += quadHeight;
		ySet.push_back(y);
	}

	meshHeight = ySet.size();
	meshWidth= xSet.size();

	xMat = MatrixXd::Zero(meshHeight, meshWidth);
	yMat = MatrixXd::Zero(meshHeight, meshWidth);
	for (int i=0;i < meshHeight; ++i)
		for (int j = 0; j < meshWidth; ++j)
		{
			xMat(i, j) = xSet[j];
			yMat(i, j) = ySet[i];
		}
}

Point Mesh::getVertex(int i, int j)
{
	assert(i >= 0 && j >= 0 && i < meshHeight && j < meshWidth);
	Point p;
	p.x = xMat(i, j);
	p.y = yMat(i, j);
	return p;
}

void Mesh::setVertex(int i, int j, Point p)
{
	assert(i >= 0 && j >= 0 && i < meshHeight && j < meshWidth);
	xMat(i, j) = p.x;
	yMat(i, j) = p.y;
}

Quad Mesh::getQuad(int i, int j) // start from 1
{
	assert(i > 0 && j > 0 && i < meshHeight && j < meshWidth);
	Point v00, v01, v10, v11;
	v00 = getVertex(i - 1, j - 1);
	v01 = getVertex(i - 1, j);
	v10 = getVertex(i, j - 1);
	v11 = getVertex(i, j);
	return Quad(v00, v01, v10, v11);
}
Mesh::Mesh()
{

}
Mesh::~Mesh()
{
}
