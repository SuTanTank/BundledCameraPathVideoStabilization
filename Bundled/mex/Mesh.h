#pragma once
#include "../Eigen/Dense"
#include "Quad.h"
#include <vector>
#include <algorithm>
using namespace Eigen;
class Mesh
{
public:
	unsigned int imgRows, imgCols;
	unsigned int meshWidth, meshHeight;
	double quadWidth, quadHeight;
	MatrixXd xMat, yMat;
	Mesh(int rows, int cols, double qH, double qW);
	Point getVertex(int i, int j);
	void setVertex(int i, int j, Point p);
	Quad getQuad(int i, int j);
	Mesh();
	~Mesh();
};

