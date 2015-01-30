#ifndef CV_LEAST_SQUARE_H
#define CV_LEAST_SQUARE_H 1

#include "cvxImage.h"
#include <map>
#include <vector>

using std::map;
using std::vector;

class CvLeastSquare
{
public:
	//solve equation (Ax=b) for x
	static bool AxbSolver(const vector<map<int, double> > &matrix_vec, const vector<double> & right_vec, vector<double> &result);
};

#endif