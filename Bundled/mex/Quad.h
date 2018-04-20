#pragma once

#include <vector>
#include <algorithm>
using namespace std;
typedef struct PointStruct
{
	double x;
	double y;
} Point;

class Quad
{
public: 
	Point v00, v01, v10, v11;
	Quad(Point c00, Point c01, Point c10, Point c11);
	bool isPointIn(Point p);
	bool isPointsIn(vector<Point>& ps);
	bool getBilinearCoordinates(Point p, double* coefficients);
	double getMinX();
	double getMaxX();
	double getMinY();
	double getMaxY();

	static bool isPointInTriangular(Point p, Point v0, Point v1, Point v2);
	static bool isPointsInTriangular(vector<Point>& ps, Point v0, Point v1, Point v2);
};