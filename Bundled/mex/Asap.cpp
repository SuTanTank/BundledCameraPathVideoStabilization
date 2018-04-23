#include "Asap.h"
#include <ctime>
namespace Asap
{
	Asap::Asap()
	{

	}

	Asap::Asap(int H, int W, double qH, double qW, double a)
	{
		imgHeight = H;
		imgWidth = W;
		quadHeight = qH;
		quadWidth = qW;
		alpha = a;
		// create two meshes
		source = Mesh(H, W, qH, qW);
		destin = Mesh(H, W, qH, qW);

		height = source.meshHeight;
		width = source.meshWidth;

		for (int index = 0; index < height * width; ++index)
		{
			x_index.push_back(index);
			y_index.push_back(index + height * width);
		}

		nSmoothConstraints = (height - 2) * (width - 2) * 16 + (2 * (width + height) - 8) * 8 + 4 * 4;
		columns = width * height * 2;
		//DataConstraints = vector<Triplet<double>>(nDataConstraints);
		//SmoothConstraints = vector<Triplet<double>>(nSmoothConstraints);
		//DataConstraints.reserve(nDataConstraints);
		//SmoothConstraints.reserve(nSmoothConstraints);
	}

	Asap::~Asap()
	{

	}

	int Asap::addControlPoints(int ncp, double *p1x, double *p1y, double *p2x, double *p2y)
	{
		// only use the ppossible good features
		VectorXd distances(ncp);
		for (int i = 0; i < ncp; ++i)
		{
			distances(i) = sqrt(pow(p1x[i] - p2x[i], 2) + pow(p1y[i] - p2y[i], 2));
		}
		double threshold = distances.mean() * 4;
		//threshold = 30;
		nDataConstraints = 0;
		for (int i = 0; i < ncp; ++i)
		{
			if (distances(i) < threshold
				&& p1x[i] > 0 && p1y[i] > 0
				&& p1x[i] < imgWidth && p1y[i] < imgHeight)
				//&& abs(p1y[i] - p2y[i]) < 50
				//&& distances(i) < 100)
			{
				Point p1, p2;
				p1.x = p1x[i];
				p1.y = p1y[i];
				p2.x = p2x[i];
				p2.y = p2y[i];
				dataElement_orgPt.push_back(p1);
				dataElement_desPt.push_back(p2);
				// find the rows and cols of the mesh
				dataElement_i.push_back((int)floor(p1.y / quadHeight));
				dataElement_j.push_back((int)floor(p1.x / quadWidth));
				Quad qd = source.getQuad(dataElement_i.back() + 1, dataElement_j.back() + 1);
				// compute the bilinear coefficients
				double coefficient[4];
				if (!qd.getBilinearCoordinates(p1, coefficient))
				{
					//perror("fail to compute coefficients");
					dataElement_orgPt.pop_back();
					dataElement_desPt.pop_back();
					dataElement_i.pop_back();
					dataElement_j.pop_back();
					return 1;
				}
				else
				{
					dataElement_V00.push_back(coefficient[0]);
					dataElement_V01.push_back(coefficient[1]);
					dataElement_V10.push_back(coefficient[2]);
					dataElement_V11.push_back(coefficient[3]);
					nDataConstraints += 2;
				}
			}
		}

		return 0;
	}

	int Asap::solve()
	{
		rowCount = 0;
		if (createSmoothCons(alpha) != 0)
		{
			cerr << "create smooth term failed" << endl;
			return 1;
		}
		auto rowCount0 = rowCount;
		for (int badncp = 1; badncp != 0; badncp = checkBadCPs()) // optimize control points
		{
			rowCount = rowCount0;
			int ncp = nDataConstraints / 2;
			//cout << ncp << endl;
			VectorXd b = VectorXd::Zero(nDataConstraints + nSmoothConstraints);
			if (createDataCons(b) != 0)
			{
				cerr << "create smooth term failed" << endl;
				return 1;
			}
			assert(rowCount == nDataConstraints + nSmoothConstraints);
			SparseMatrix<double> A(rowCount, columns);			
			//auto start = clock();
			vector<T> allConstraints;
			allConstraints.insert(allConstraints.end(), SmoothConstraints.begin(), SmoothConstraints.end());
			allConstraints.insert(allConstraints.end(), DataConstraints.begin(), DataConstraints.end());			
			A.setFromTriplets(allConstraints.begin(), allConstraints.end());
			VectorXd x = VectorXd::Zero(A.cols());
			VectorXd x0 = VectorXd::Zero(A.cols());			

			for (int i = 0; i < height; ++i)
				for (int j = 0; j < width; ++j)
				{
					auto index = i * width + j;
					auto P = destin.getVertex(i, j);
					x0(x_index[index]) = P.x;
					x0(y_index[index]) = P.y;
				}

			LeastSquaresConjugateGradient<SparseMatrix<double> > lscg;
			lscg.compute(A);
			x = lscg.solveWithGuess(b, x0);
			//auto duration = (clock() - start) / (double)CLOCKS_PER_SEC;
			//cout << "duration:" << duration << endl;
			//cout << lscg.error() << endl;
			//SparseQR<SparseMatrix<double>, COLAMDOrdering<int> > sQR;
			//sQR.compute(A);
			//x = sQR.solve(b);
			//cout << (x - x1).norm() << endl;

			for (int i = 0; i < height; ++i)
				for (int j = 0; j < width; ++j)
				{
					Point p;
					p.x = x(i * width + j);
					p.y = x(i * width + j + columns / 2);
					destin.setVertex(i, j, p);
				}
			
		}
		//checkBadCPs();
		return 0;
	}

	vector<vector<Homography>> Asap::calcHomos()
	{
		vector<vector<Homography>> homos;		
		for (int row = 0; row < height - 1; ++row)
		{
			vector<Homography> homoRow;
			for (int col = 0; col < width - 1; ++col)
			{
				Quad q1 = source.getQuad(row + 1, col + 1);
				Quad q2 = destin.getQuad(row + 1, col + 1);
				// TODO:
				// get 4 pairs of points and compute the homography
				Homography h;
				vector<Point> p1, p2;
				p1.push_back(q1.v00);
				p1.push_back(q1.v01);
				p1.push_back(q1.v10);
				p1.push_back(q1.v11);
				p2.push_back(q2.v00);
				p2.push_back(q2.v01);
				p2.push_back(q2.v10);
				p2.push_back(q2.v11);
				h.computeFromPoints(p1, p2);
				h.normalize();
				homoRow.push_back(h);
			}
			homos.push_back(homoRow);
		}
		return homos;
	}

	double Asap::CalcError()
	{
		auto ncp = nDataConstraints / 2;
		double error = 0;
		for (int i = 0; i < ncp; ++i)
		{
			auto q = destin.getQuad(dataElement_i[i] + 1, dataElement_j[i] + 1);
			auto term_x = q.v00.x * dataElement_V00[i]
				+ q.v01.x * dataElement_V01[i]
				+ q.v10.x * dataElement_V10[i]
				+ q.v11.x * dataElement_V11[i]
				- dataElement_desPt[i].x;
			auto term_y = q.v00.y * dataElement_V00[i]
				+ q.v01.y * dataElement_V01[i]
				+ q.v10.y * dataElement_V10[i]
				+ q.v11.y * dataElement_V11[i]
				- dataElement_desPt[i].y;
			error += sqrt(term_x * term_x + term_y * term_y);
		}
		return error / ncp;
	}

	int Asap::createDataCons(VectorXd& b)
	{
		DataConstraints.clear();
		for (int k = 0; k < nDataConstraints / 2; ++k)
		{
			//cout << dataElement_orgPt[k].x << '\t' << dataElement_orgPt[k].y << endl;			
			int i = dataElement_i[k];
			int j = dataElement_j[k];
			//if (i >= height - 2 || j >= width - 2)
			//{
			//	printf("?");
			//}

			double v00 = dataElement_V00[k];
			double v01 = dataElement_V01[k];
			double v10 = dataElement_V10[k];
			double v11 = dataElement_V11[k];

			int index00 = i * width + j;
			int index01 = i * width + j + 1;
			int index10 = (i + 1) * width + j;
			int index11 = (i + 1) * width + j + 1;
			DataConstraints.push_back(T(rowCount, x_index[index00], v00));
			DataConstraints.push_back(T(rowCount, x_index[index01], v01));
			DataConstraints.push_back(T(rowCount, x_index[index10], v10));
			DataConstraints.push_back(T(rowCount, x_index[index11], v11));
			b(rowCount) = dataElement_desPt[k].x;
			++rowCount;

			DataConstraints.push_back(T(rowCount, y_index[index00], v00));
			DataConstraints.push_back(T(rowCount, y_index[index01], v01));
			DataConstraints.push_back(T(rowCount, y_index[index10], v10));
			DataConstraints.push_back(T(rowCount, y_index[index11], v11));
			b(rowCount) = dataElement_desPt[k].y;
			++rowCount;
		}
		return 0;
	}

	int Asap::checkBadCPs()
	{
		int ncp = nDataConstraints / 2;
		int badncp = 0;
		VectorXd distances(ncp);
		double count = 0;
		double sum = 0;
		for (int i = 0; i < ncp; ++i)
		{			
			auto q = destin.getQuad(dataElement_i[i] + 1, dataElement_j[i] + 1);
			auto term_x = q.v00.x * dataElement_V00[i]
				+ q.v01.x * dataElement_V01[i]
				+ q.v10.x * dataElement_V10[i]
				+ q.v11.x * dataElement_V11[i]
				- dataElement_desPt[i].x;
			auto term_y = q.v00.y * dataElement_V00[i]
				+ q.v01.y * dataElement_V01[i]
				+ q.v10.y * dataElement_V10[i]
				+ q.v11.y * dataElement_V11[i]
				- dataElement_desPt[i].y;
			distances(i) = sqrt(term_x * term_x + term_y * term_y);
			if (distances(i) > 1)
			{
				sum += distances(i);
				count++;
			}
		}
		double threshold = max(sum/(count + 0.00001) * 5, 2.0);					
		assert(dataElement_i.size() == ncp);
		vector<int> _dataElement_i;
		vector<int> _dataElement_j;
		vector<double> _dataElement_V00;
		vector<double> _dataElement_V01;
		vector<double> _dataElement_V10;
		vector<double> _dataElement_V11;
		vector<Point> _dataElement_orgPt;
		vector<Point> _dataElement_desPt;
		for (int i = 0; i < ncp; ++i)
		{			
			if (distances(i) < threshold)				
			{	
				_dataElement_orgPt.push_back(dataElement_orgPt[i]);
				_dataElement_desPt.push_back(dataElement_desPt[i]);
				_dataElement_i.push_back(dataElement_i[i]);
				_dataElement_j.push_back(dataElement_j[i]);
				
				_dataElement_V00.push_back(dataElement_V00[i]);
				_dataElement_V01.push_back(dataElement_V01[i]);
				_dataElement_V10.push_back(dataElement_V10[i]);
				_dataElement_V11.push_back(dataElement_V11[i]);				
			}
			else
			{
				nDataConstraints -= 2;
				badncp++;
			}
		}
		dataElement_desPt = _dataElement_desPt;
		dataElement_orgPt = _dataElement_orgPt;
		dataElement_i = _dataElement_i;
		dataElement_j = _dataElement_j;
		dataElement_V00 = _dataElement_V00;
		dataElement_V01 = _dataElement_V01;
		dataElement_V10 = _dataElement_V10;
		dataElement_V11 = _dataElement_V11;
		//cout << "bads: " << badncp << "Thre" << threshold << endl;
		return badncp;
	}

	int Asap::createSmoothCons(double weight)
	{
		rowCount = 0;
		SmoothConstraints.clear();
		for (int i = 0; i < height; ++i)
			for (int j = 0; j < width; ++j)
			{
				// i: row, j: col
				addCoefficient_LU(i, j);
				addCoefficient_UL(i, j);
				addCoefficient_UR(i, j);
				addCoefficient_RU(i, j);
				addCoefficient_RD(i, j);
				addCoefficient_DR(i, j);
				addCoefficient_DL(i, j);
				addCoefficient_LD(i, j);
			}
		return 0;
	}

	string Asap::printInfo()
	{
		stringstream info;
		info << "Info..\n";
		info << "Number of valid Control Points: " << dataElement_orgPt.size() << endl;
		return info.str();
	}



	void Asap::assignUV(Point v1, Point v2, Point v3, double & u, double & v)
	{
		double d1 = sqrt((v1.x - v2.x) * (v1.x - v2.x) + (v1.y - v2.y)*(v1.y - v2.y));
		double d3 = sqrt((v2.x - v3.x) * (v2.x - v3.x) + (v2.y - v3.y)*(v2.y - v3.y));

		Point v21, v23;
		v21.x = v1.x - v2.x;
		v21.y = v1.y - v2.y;
		v23.x = v3.x - v2.x;
		v23.y = v3.y - v2.y;

		double cosin = (v21.x * v23.x + v21.y * v23.y) / (d1 * d3);
		u = cosin * d1 / d3;
		v = sqrt(d1 * d1 - cosin * d1 * cosin * d1) / d3;
	}

	void Asap::addCoefficient_LU(int i, int j)
	{
		if (i == 0 || j == 0)
		{
			return;
		}
		Point v1 = source.getVertex(i, j);
		Point v2 = source.getVertex(i, j - 1);
		Point v3 = source.getVertex(i - 1, j - 1);
		double u, v;
		assignUV(v1, v2, v3, u, v);

		int index1 = i * width + j;
		int index2 = i * width + j - 1;
		int index3 = (i - 1) * width + j - 1;

		SmoothConstraints.push_back(T(rowCount, x_index[index2], (1 - u) * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index3], u * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index2], v * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index3], -v * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index1], -alpha));
		++rowCount;

		SmoothConstraints.push_back(T(rowCount, y_index[index2], (1 - u) * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index3], u * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index3], v * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index2], -v * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index1], -alpha));
		++rowCount;
	}

	void Asap::addCoefficient_UL(int i, int j)
	{
		//            V3   V2
		//             _____
		//					|
		//					|
		//                 V1(i, j)
		if (i == 0 || j == 0)
		{
			return;
		}
		Point v1 = source.getVertex(i, j);
		Point v2 = source.getVertex(i - 1, j);
		Point v3 = source.getVertex(i - 1, j - 1);
		double u, v;
		assignUV(v1, v2, v3, u, v);

		int index1 = i * width + j;
		int index2 = (i - 1) * width + j;
		int index3 = (i - 1) * width + j - 1;

		SmoothConstraints.push_back(T(rowCount, x_index[index2], (1 - u) * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index3], u * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index3], v * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index2], -v * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index1], -alpha));
		++rowCount;

		SmoothConstraints.push_back(T(rowCount, y_index[index2], (1 - u) * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index3], u * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index2], v * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index3], -v * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index1], -alpha));
		++rowCount;
	}

	void Asap::addCoefficient_UR(int i, int j)
	{
		if (i == 0 || j == width - 1)
		{
			return;
		}
		Point v1 = source.getVertex(i, j);
		Point v2 = source.getVertex(i - 1, j);
		Point v3 = source.getVertex(i - 1, j + 1);
		double u, v;
		assignUV(v1, v2, v3, u, v);

		int index1 = i * width + j;
		int index2 = (i - 1) * width + j;
		int index3 = (i - 1) * width + j + 1;

		SmoothConstraints.push_back(T(rowCount, x_index[index2], (1 - u) * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index3], u * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index2], v * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index3], -v * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index1], -alpha));
		++rowCount;

		SmoothConstraints.push_back(T(rowCount, y_index[index2], (1 - u) * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index3], u * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index3], v * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index2], -v * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index1], -alpha));
		++rowCount;
	}

	void Asap::addCoefficient_RU(int i, int j)
	{
		if (i == 0 || j == width - 1)
		{
			return;
		}
		Point v1 = source.getVertex(i, j);
		Point v2 = source.getVertex(i, j + 1);
		Point v3 = source.getVertex(i - 1, j + 1);
		double u, v;
		assignUV(v1, v2, v3, u, v);

		int index1 = i * width + j;
		int index2 = i * width + j + 1;
		int index3 = (i - 1) * width + j + 1;

		SmoothConstraints.push_back(T(rowCount, x_index[index2], (1 - u) * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index3], u * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index3], v * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index2], -v * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index1], -alpha));
		++rowCount;

		SmoothConstraints.push_back(T(rowCount, y_index[index2], (1 - u) * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index3], u * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index2], v * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index3], -v * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index1], -alpha));
		++rowCount;
	}

	void Asap::addCoefficient_RD(int i, int j)
	{
		if (i == height - 1 || j == width - 1)
		{
			return;
		}
		Point v1 = source.getVertex(i, j);
		Point v2 = source.getVertex(i, j + 1);
		Point v3 = source.getVertex(i + 1, j + 1);
		double u, v;
		assignUV(v1, v2, v3, u, v);

		int index1 = i * width + j;
		int index2 = i * width + j + 1;
		int index3 = (i + 1) * width + j + 1;

		SmoothConstraints.push_back(T(rowCount, x_index[index2], (1 - u) * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index3], u * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index2], v * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index3], -v * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index1], -alpha));
		++rowCount;

		SmoothConstraints.push_back(T(rowCount, y_index[index2], (1 - u) * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index3], u * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index3], v * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index2], -v * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index1], -alpha));
		++rowCount;
	}

	void Asap::addCoefficient_DR(int i, int j)
	{
		if (i == height - 1 || j == width - 1)
		{
			return;
		}
		Point v1 = source.getVertex(i, j);
		Point v2 = source.getVertex(i + 1, j);
		Point v3 = source.getVertex(i + 1, j + 1);
		double u, v;
		assignUV(v1, v2, v3, u, v);

		int index1 = i * width + j;
		int index2 = (i + 1) * width + j;
		int index3 = (i + 1) * width + j + 1;

		SmoothConstraints.push_back(T(rowCount, x_index[index2], (1 - u) * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index3], u * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index3], v * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index2], -v * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index1], -alpha));
		++rowCount;

		SmoothConstraints.push_back(T(rowCount, y_index[index2], (1 - u) * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index3], u * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index2], v * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index3], -v * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index1], -alpha));
		++rowCount;
	}

	void Asap::addCoefficient_DL(int i, int j)
	{
		if (i == height - 1 || j == 0)
		{
			return;
		}
		Point v1 = source.getVertex(i, j);
		Point v2 = source.getVertex(i + 1, j);
		Point v3 = source.getVertex(i + 1, j - 1);
		double u, v;
		assignUV(v1, v2, v3, u, v);

		int index1 = i * width + j;
		int index2 = (i + 1) * width + j;
		int index3 = (i + 1) * width + j - 1;

		SmoothConstraints.push_back(T(rowCount, x_index[index2], (1 - u) * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index3], u * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index2], v * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index3], -v * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index1], -alpha));
		++rowCount;

		SmoothConstraints.push_back(T(rowCount, y_index[index2], (1 - u) * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index3], u * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index3], v * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index2], -v * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index1], -alpha));
		++rowCount;
	}

	void Asap::addCoefficient_LD(int i, int j)
	{
		if (i == height - 1 || j == 0)
		{
			return;
		}
		Point v1 = source.getVertex(i, j);
		Point v2 = source.getVertex(i, j - 1);
		Point v3 = source.getVertex(i + 1, j - 1);
		double u, v;
		assignUV(v1, v2, v3, u, v);

		int index1 = i * width + j;
		int index2 = i * width + j - 1;
		int index3 = (i + 1) * width + j - 1;

		SmoothConstraints.push_back(T(rowCount, x_index[index2], (1 - u) * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index3], u * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index3], v * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index2], -v * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index1], -alpha));
		++rowCount;

		SmoothConstraints.push_back(T(rowCount, y_index[index2], (1 - u) * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index3], u * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index2], v * alpha));
		SmoothConstraints.push_back(T(rowCount, x_index[index3], -v * alpha));
		SmoothConstraints.push_back(T(rowCount, y_index[index1], -alpha));
		++rowCount;
	}

	Homography::Homography()
	{
		mat = Matrix3d::Identity(3, 3);
		trans = mat;
	}

	Homography::~Homography()
	{
	}

	Homography::Homography(Matrix3d & homo)
	{
		mat = homo;
		trans = homo;
	}

	Homography::Homography(vector<double>& n9)
	{
		assert(n9.size() == 9);
		double data[9];
		for (int i = 0; i < 9; ++i)
		{
			data[i] = n9[i];
		}
		Map<Matrix3d> mat(data);
		trans = mat;
	}

	Homography::Homography(double data[9])
	{
		Map<Matrix3d> mat(data);
	}

	Matrix3d Homography::getMat()
	{
		return mat;
	}

	Transform<double, 2, Projective> Homography::getTransfrom()
	{
		return trans;
	}

	vector<double> Homography::getN9()
	{
		vector<double> n9;
		for (auto i = 0; i < 9; ++i)
			n9[i] = mat(i % 3, i / 3);
		return n9;
	}

	int Homography::computeFromPoints(vector<Point> p1, vector<Point> p2)
	{
		assert(p1.size() == p2.size());
		assert(p1.size() >= 4);
		auto np = p1.size();
		MatrixXd rows0 = MatrixXd::Zero(3, np);
		MatrixXd rowsXY = MatrixXd::Zero(3, np);
		VectorXd p1x(np), p1y(np), p2x(np), p2y(np);
		for (int i = 0; i < np; ++i)
		{
			p1x(i) = p1[i].x;
			p1y(i) = p1[i].y;
			p2x(i) = p2[i].x;
			p2y(i) = p2[i].y;
		}
		/*
		x = pout(1, :); y = pout(2,:); X = pin(1,:); Y = pin(2,:);
		rows0 = zeros(3, n);
		rowsXY = -[X; Y; ones(1,n)];
		hx = [rowsXY; rows0; x.*X; x.*Y; x];
		hy = [rows0; rowsXY; y.*X; y.*Y; y];
		h = [hx hy];
		*/

		rowsXY.block(0, 0, 1, np) = p1x.transpose() * -1;
		rowsXY.block(1, 0, 1, np) = p1y.transpose() * -1;
		rowsXY.block(2, 0, 1, np) = VectorXd::Ones(np).transpose() * -1;
		//cout << rowsXY << endl;
		MatrixXd h(9, np * 2);
		h.block(0, 0, 3, np) = rowsXY;
		h.block(3, 0, 3, np) = rows0;
		h.block(0, np, 3, np) = rows0;
		h.block(3, np, 3, np) = rowsXY;
		h.block(6, 0, 1, np) = p1x.cwiseProduct(p2x).transpose();
		h.block(7, 0, 1, np) = p1y.cwiseProduct(p2x).transpose();
		h.block(8, 0, 1, np) = p2x.transpose();
		h.block(6, np, 1, np) = p1x.cwiseProduct(p2y).transpose();
		h.block(7, np, 1, np) = p1y.cwiseProduct(p2y).transpose();
		h.block(8, np, 1, np) = p2y.transpose();

		//cout << h << endl;
		JacobiSVD<MatrixXd> svd(h, ComputeFullU | ComputeThinV);
		MatrixXd U9 = svd.matrixU().block(0, 8, 9, 1);
		U9.resize(3, 3);
		mat = U9.transpose();
		//cout << mat << endl;
		trans = mat;
		return 0;
	}

	void Homography::normalize()
	{
		mat *= 1 / mat(2, 2);
		trans = mat;
	}
}


