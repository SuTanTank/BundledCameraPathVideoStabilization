#include "Quad.h"
Quad::Quad(Point c00, Point c01, Point c10, Point c11)
{
	v00 = c00; //left-up
	v01 = c01; //right-up
	v10 = c10; //left-down
	v11 = c11; //right-down
}

bool Quad::isPointIn(Point p)
{
	bool in1 = isPointInTriangular(p, v00, v01, v11);
	bool in2 = isPointInTriangular(p, v00, v10, v11);
	return in1 | in2;
}

bool Quad::isPointsIn(vector<Point>& ps)
{
	// maybe not needed	
	return false;
}

bool Quad::getBilinearCoordinates(Point p, double *coefficients)
{
	double x1 = v00.x;
	if (x1 != v10.x) 
	{
		perror("not rectangle?");
		return false;
	}
	double x2 = v01.x;
	if (x2 != v11.x)
	{
		perror("not rectangle?");
		return false;
	}
	double y1 = v00.y;
	if (y1 != v01.y)
	{
		perror("not rectangle?");
		return false;
	}
	double y2 = v10.y;
	if (y2 != v11.y)
	{
		perror("not rectangle?");
		return false;
	}
	if (!isPointIn(p))
	{
		// perror("not in quad");
		// bool xxx = isPointIn(p);
		return false;
	}
	else
	{
		coefficients[0] = (x2 - p.x) * (y2 - p.y) / (x2 - x1) / (y2 - y1);
		coefficients[1] = (p.x - x1) * (y2 - p.y) / (x2 - x1) / (y2 - y1);
		coefficients[2] = (x2 - p.x) * (p.y - y1) / (x2 - x1) / (y2 - y1);
		coefficients[3] = (p.x - x1) * (p.y - y1) / (x2 - x1) / (y2 - y1);
	}
	return true;
}

double Quad::getMinX()
{
	double t = min(v01.x, v00.x);
	t = min(t, v10.x);
	return min(t, v11.x);
}

double Quad::getMaxX()
{
	double t = max(v01.x, v00.x);
	t = max(t, v10.x);
	return max(t, v11.x);
}

double Quad::getMinY()
{
	double t = min(v01.y, v00.y);
	t = min(t, v10.y);
	return min(t, v11.y);
}

double Quad::getMaxY()
{
	double t = max(v01.y, v00.y);
	t = max(t, v10.y);
	return max(t, v11.y);
}

bool Quad::isPointInTriangular(Point p, Point v0, Point v1, Point v2)
{
	/*
	a = ((v1.y - v2.y) * (p.x - v2.x) + (v2.x - v1.x) * (p.y - v2.y)) / ((v1.y - v2.y) * (v0.x - v2.x) + (v2.x - v1.x) * (v0.y - v2.y));
	b = ((v2.y - v0.y) * (p.x - v2.x) + (v0.x - v2.x) * (p.y - v2.y)) / ((v1.y - v2.y) * (v0.x - v2.x) + (v2.x - v1.x) * (v0.y - v2.y));
	c = 1 - a - b
	*/
	double lambda1 = ((v1.y - v2.y) * (p.x - v2.x) + (v2.x - v1.x) * (p.y - v2.y)) / ((v1.y - v2.y) * (v0.x - v2.x) + (v2.x - v1.x) * (v0.y - v2.y));
	double lambda2 = ((v2.y - v0.y) * (p.x - v2.x) + (v0.x - v2.x) * (p.y - v2.y)) / ((v1.y - v2.y) * (v0.x - v2.x) + (v2.x - v1.x) * (v0.y - v2.y));
	double lambda3 = 1 - lambda1 - lambda2;	
	return lambda1 >= 0 & lambda1 <= 1 & lambda2 >= 0 & lambda2 <= 1 & lambda3 >= 0 & lambda3 <= 1;
}

bool Quad::isPointsInTriangular(vector<Point>& ps, Point v0, Point v1, Point v2)
{
	return false;
}
