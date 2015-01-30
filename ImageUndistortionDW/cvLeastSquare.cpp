#include "cvLeastSquare.h"

//solve (Ax=b) for x
bool CvLeastSquare::AxbSolver(const vector<map<int, double> > &matrix_vec, const vector<double> & right_vec, vector<double> &result)
{
	assert(matrix_vec.size() == right_vec.size());
	assert(result.size() != 0);

	//copy data to A and b
	Mat A = Mat::zeros(matrix_vec.size(), result.size(), CV_64F);
	Mat b = Mat::zeros(matrix_vec.size(), 1, CV_64F);
	
	for(int i = 0; i<matrix_vec.size(); ++i)
	{
		for(map<int, double>::const_iterator j = matrix_vec[i].begin(); j!=matrix_vec[i].end(); ++j)
		{
			A.at<double>(i, j->first) = j->second;
		}
	}
	for(int i = 0; i<right_vec.size(); ++i)
	{
		b.at<double>(i) = right_vec[i];
	}
	Mat x = Mat(result.size(), 1, CV_64F, &result[0]);	
	x = (A.t()*A).inv()*(A.t()*b);		
	return true;
}