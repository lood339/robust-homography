#include "cvxRobustHomoImageUndistortion.h"
#include "markerchessboard.hpp"
#include "cvLeastSquare.h"

using namespace::cv;
using cvx::Bw32FImage;


CvxRobustHomoImageUndistortion::CvxRobustHomoImageUndistortion()
{
	m_lambda = 0.0;
	m_scale  = -1.0;
	pBoard = NULL;
}

CvxRobustHomoImageUndistortion::~CvxRobustHomoImageUndistortion()
{
	if(pBoard)
	{
		delete pBoard;
		pBoard = NULL;
	}
}



void CvxRobustHomoImageUndistortion::setBoardSize(const cv::Size &patternSize)
{
	pBoard = new CvMarkerChessboard(patternSize);
	assert(pBoard);
}



bool CvxRobustHomoImageUndistortion::estimateDistortionParameter(const Mat &inputImage, const cv::Size &patternSize, 
												bool isTracking, double noiseStdDevToCorners, bool showResult)
{
	//check input
	assert(!inputImage.empty());
	assert(patternSize.width >= 3 && patternSize.height >= 3);

	//find corner position
	bool isFound = false;
	vector<Point2f> boardPts;
	vector<Point2f> corners;
	if(pBoard)
	{
		isFound = pBoard->findCorners(inputImage, isTracking, corners, 10);	
		if(!isFound)
		{
			printf("Board can not find corners.\n");			
		}
	}
	else
	{
		CvMarkerChessboard board(patternSize);
		isFound = board.findCorners(inputImage, false, corners, 10);
	}	

	if(!isFound)
	{
	//	printf("mark chessboard corner not found\n");
		isFound = cv::findChessboardCorners(inputImage, patternSize, corners);
		if(!isFound)
		{
			printf("Open CV can not found corners. Please check the input image.\n");
			return false;
		}		
	}

	//add noise to corners position, only used in the test
	if(noiseStdDevToCorners != 0.0)
	{
		Mat noiseX = Mat::zeros(corners.size(), 1, CV_64F);	
		Mat noiseY = Mat::zeros(corners.size(), 1, CV_64F);	
		cv::randn(noiseX, Scalar::all(0.0), Scalar::all(noiseStdDevToCorners));
		cv::randn(noiseY, Scalar::all(0.0), Scalar::all(noiseStdDevToCorners));
		double *pNoiseX = (double *) noiseX.data;
		double *pNoiseY = (double *) noiseY.data;
		for(unsigned int i = 0; i<corners.size(); i++)
		{
			corners[i].x += pNoiseX[i];
			corners[i].y += pNoiseY[i];
		}
	}

	//generate corner positin on the board
	for(int y = 0; y<patternSize.height; y++)
	{
		for(int x = 0; x<patternSize.width; x++)
		{
			Point2f p(1.0f * x, 1.0f * y);
			boardPts.push_back(p);
		}
	}

	//normalize corner position on the board
	{
		Point2d boardCenterPt;
		double boardScale = 0.0;
		CvxRobustHomoImageUndistortion::getNormalizedCenterAndScale(boardPts, boardCenterPt, boardScale);

		for(unsigned int i = 0; i<boardPts.size(); i++)
		{
			boardPts[i].x = (boardPts[i].x - boardCenterPt.x) * boardScale;
			boardPts[i].y = (boardPts[i].y - boardCenterPt.y) * boardScale;
		}
	}

	//normalize corner positions in the image
	
	{
		m_normalizeCenter.x = 0.0; 
		m_normalizeCenter.y = 0.0;
		m_scale = 0.0;

		// set the image center as distortion center
		m_imageCenter.x = inputImage.cols/2.0 - 0.5;
		m_imageCenter.y = inputImage.rows/2.0 - 0.5;

		for(unsigned int i = 0; i<corners.size(); i++)
		{
			corners[i].x = corners[i].x - m_imageCenter.x;
			corners[i].y = corners[i].y - m_imageCenter.y;
		}	
		
		CvxRobustHomoImageUndistortion::getNormalizedCenterAndScale(corners, m_normalizeCenter, m_scale);

		// change corners to the normalized space and shift the center
		for(unsigned int i = 0; i<corners.size(); i++)
		{
			corners[i].x = (corners[i].x - m_normalizeCenter.x) * m_scale;
			corners[i].y = (corners[i].y - m_normalizeCenter.y) * m_scale;
		}		
	}
	
	//estimate parameter from normalized corners
	bool isOk = this->estimateDistortionParameter(boardPts, corners);
	if(isOk && showResult)
	{
		printf("lambda is %f\n", m_lambda);
	}	
	return isOk;
}

bool CvxRobustHomoImageUndistortion::estimateDistortionParameter(const vector<Point2f> &boardCorners, const vector<Point2f> &imageCorners)
{
	assert(boardCorners.size() == imageCorners.size());
	assert(boardCorners.size() >= 64);   //100-200 will be better

	// get initial result of homography matrix
	Mat H;	
	H = cv::findHomography(boardCorners, imageCorners);

	vector<double> residence(9, 0.0); //estimation result, 0-7, is homography matrix, 8 is lambda
	double *pH = (double *)H.data;
	for(int i = 0; i<8; i++)
	{
		residence[i] = pH[i];
	}

	int iter_num = 5;               //iternation number
	for(int i = 0; i<iter_num; i++)
	{
		vector<double> result(9);
		bool isOk = false;
		isOk = CvxRobustHomoImageUndistortion::solveOnce(residence, boardCorners, imageCorners, result);
		
		//iteratively update result
		if(isOk)
		{
			for(unsigned int i = 0; i<result.size(); i++)
			{
				residence[i] = residence[i] + result[i];
			}
		}
		else
		{
			printf("iteration failed.\n");
			return false;
		}		
	}
	m_lambda = residence[8];	
	return true;
}

bool CvxRobustHomoImageUndistortion::solveOnce(const vector<double> &residence, const vector<Point2f> &boardPts, 
		                  const vector<Point2f> &corners, vector<double> &result)
{
	assert(residence.size() == 9);
	assert(boardPts.size() == corners.size());
	assert(result.size() == 9);

	// iterative Levenberg-Marquartd algorithm	
	vector<map<int, double>> leftVec;
	vector<double> rightVec;
	for(int i = 0; i<boardPts.size(); i++)
	{
		Point2f p1 = boardPts[i];
		Point2f p2 = corners[i];

		double x = p2.x;
		double y = p2.y;
		double s = p1.x;
		double t = p1.y;
		double k = x * x + y * y;

		double A = residence[0];
		double B = residence[1];
		double C = residence[2];
		double D = residence[3];
		double E = residence[4];
		double F = residence[5];
		double G = residence[6];
		double H = residence[7];
		double L = residence[8];  //lambda

		{
			map<int, double> imap;
			double v_right = 0.0f;
			imap[0] = s + k * s * L;
			imap[1] = t + k * t * L;
			imap[2] = 1.0 + k * L;
			imap[6] = - x * s;
			imap[7] = - x * t;
			imap[8] = k * s * A + k * t * B + k * C;
			v_right = x - (s*A + t*B + C + k*s*A*L + k*t*B*L + k*C*L - x*s*G - x*t*H);
			leftVec.push_back(imap);
			rightVec.push_back(v_right);		
		}

		{
			map<int, double> imap;
			double v_right = 0.0f;
			imap[3] = s + k*s*L;
			imap[4] = t + k*t*L;
			imap[5] = 1.0 + k*L;
			imap[6] = -y*s;
			imap[7] = -y*t;
			imap[8] = k*s*D + k*t*E + k*F;
			v_right = y - (s*D + t*E + F + k*s*D*L + k*t*E*L + k*F*L - y*s*G - y*t*H);
			leftVec.push_back(imap);
			rightVec.push_back(v_right);
		}
	}
	
	bool isOk = CvLeastSquare::AxbSolver(leftVec, rightVec, result);
	return isOk;
}


Mat CvxRobustHomoImageUndistortion::distortImage(const Mat &image, const cv::Size &patternSize, double lambda)
{
	//check input
	assert(!image.empty());
	int h = image.rows;
	int w = image.cols;

	Mat retMat = cv::Mat::zeros(h, w, CV_8UC3);
	Point2d normalizeCenter;
	Point2d distortionCenter;
	double scale = 0.0;

	//find corner position
	CvMarkerChessboard board(patternSize);
	vector<Point2f> corners;
	bool isFound = board.findCorners(image, false, corners, 10);
	if(!isFound)
	{
		printf("corner not found\n");
		return retMat;		
	}

	// change image center to normalized space
	distortionCenter.x = w/2.0 - 0.5;
	distortionCenter.y = h/2.0 - 0.5;

	for(int i = 0; i<corners.size(); i++)
	{
		corners[i].x = corners[i].x - distortionCenter.x;
		corners[i].y = corners[i].y - distortionCenter.y;
	}

	//normalize data
	scale = 0.0;
	CvxRobustHomoImageUndistortion::getNormalizedCenterAndScale(corners, normalizeCenter, scale);

	Mat remapX = cv::Mat::zeros(h, w, CV_32FC1);
	Mat remapY = cv::Mat::zeros(h, w, CV_32FC1);

	Bw32FImage rem_x(&remapX);
	Bw32FImage rem_y(&remapY);
	//distort in normalized space
	for(int y = 0; y<h; y++)
	{
		for(int x = 0; x<w; x++)
		{
			Point2d p(1.0*x, 1.0*y);
			Point2d q;  //perfect position

			p.x = p.x - distortionCenter.x;
			p.y = p.y - distortionCenter.y;

			p.x = (p.x - normalizeCenter.x) * scale;
			p.y = (p.y - normalizeCenter.y) * scale;

			double dis_square = p.x * p.x + p.y * p.y;
			q.x = 1.0/(1.0 + lambda * dis_square) * p.x;
			q.y = 1.0/(1.0 + lambda * dis_square) * p.y;

			//change back to image space
			q.x = q.x/scale + normalizeCenter.x;
			q.y = q.y/scale + normalizeCenter.y;

			q.x = q.x + distortionCenter.x;
			q.y = q.y + distortionCenter.y;

			rem_x[y][x] = q.x;
			rem_y[y][x] = q.y;			
		}
	}
	cv::remap(image, retMat, remapX, remapY, INTER_LINEAR, BORDER_CONSTANT);
	return retMat;
}

Mat CvxRobustHomoImageUndistortion::undistortImage(const Mat &dImage)
{
	//check input
	assert(!dImage.empty());
	int h = dImage.rows;
	int w = dImage.cols;

	Mat retMat = cv::Mat::zeros(h, w, CV_8UC3);
	Mat remapX = cv::Mat::zeros(h, w, CV_32FC1);
	Mat remapY = cv::Mat::zeros(h, w, CV_32FC1);

	//reverse
	Bw32FImage rem_x(&remapX);
	Bw32FImage rem_y(&remapY);
	for(int y = 0; y< h; y++)
	{
		for(int x = 0;x <w; x++)
		{
			//change to normalized positon
			Point2d p(1.0 * x, 1.0 *y);  //perfect postion
			Point2d q;                   //position in distorted image
 
			p.x = p.x - m_imageCenter.x;
			p.y = p.y - m_imageCenter.y;

			p.x = (p.x - m_normalizeCenter.x) * m_scale;
			p.y = (p.y - m_normalizeCenter.y) * m_scale;

			double v_t = 1.0 - 4 * m_lambda * p.x * p.x - 4 * m_lambda * p.y * p.y;
			double u_t = 2.0 * m_lambda * p.x * p.x + 2.0 * m_lambda * p.y * p.y;

			if(v_t >= 0.0 && u_t != 0.0)
			{
				v_t = sqrt(v_t);
				q.x = (p.x - p.x * v_t)/u_t;
				q.y = (p.y - p.y * v_t)/u_t;

				q.x = q.x/m_scale + m_normalizeCenter.x;
				q.y = q.y/m_scale + m_normalizeCenter.y;

				//shift back distortion center
				q.x = q.x + m_imageCenter.x;
				q.y = q.y + m_imageCenter.y;

				rem_x[y][x] = q.x;
				rem_y[y][x] = q.y;	

			}
		}
	}

	cv::remap(dImage, retMat, remapX, remapY, INTER_LINEAR, BORDER_CONSTANT);   // BORDER_TRANSPARENT
	return retMat;
}

Mat CvxRobustHomoImageUndistortion::undistortImageWithScale(const Mat &dImage)
{
	assert(!dImage.empty());

	if(m_lambda > 0.0)
	{
		return this->undistortImage(dImage);
	}
	const int h = dImage.rows;
	const int w = dImage.cols;

	//calculate the scale from four image corners
	vector<Point2f> fourCorners;
	fourCorners.push_back(Point2f(0, 0));
	fourCorners.push_back(Point2f(w-1, 0));
	fourCorners.push_back(Point2f(0, h-1));
	fourCorners.push_back(Point2f(w-1, h-1));

	vector<Point2f> unDistortedFourCorners = fourCorners;
	this->undistortPoints(unDistortedFourCorners);

	Point2f p_min(INT_MAX, INT_MAX);
	Point2f p_max(0, 0);
	for(int i = 0; i<unDistortedFourCorners.size(); i++)
	{
		float x = unDistortedFourCorners[i].x;
		float y = unDistortedFourCorners[i].y;

		p_min.x = std::min(x, p_min.x);
		p_min.y = std::min(y, p_min.y);

		p_max.x = std::max(x, p_max.x);
		p_max.y = std::max(y, p_max.y);
	}
	float scaleX = 1.0 *(p_max.x - p_min.x)/w;
	float scaleY = 1.0 *(p_max.y - p_min.y)/h;
	float scale = std::max(scaleX, scaleY);
	int sW = scale * w;
	int sH = scale * h;

	//enlarged image
	Mat sImage = cv::Mat::zeros(sH, sW, CV_8UC3);
	Mat remapX = cv::Mat::zeros(sH, sW, CV_32FC1);
	Mat remapY = cv::Mat::zeros(sH, sW, CV_32FC1);

	Point2d imageCenter;
	imageCenter.x = sW/2.0 - 0.5;
	imageCenter.y = sH/2.0 - 0.5;

	//reverse
	Bw32FImage rem_x(&remapX);
	Bw32FImage rem_y(&remapY);
	for(int y = 0; y< sH; y++)
	{
		for(int x = 0;x <sW; x++)
		{
			//change to normalized positon
			Point2d p(1.0 * x, 1.0 *y);  //perfect postion
			Point2d q;                   //position in distorted image
 
			p.x = p.x - imageCenter.x;
			p.y = p.y - imageCenter.y;

			p.x = (p.x - m_normalizeCenter.x) * m_scale;
			p.y = (p.y - m_normalizeCenter.y) * m_scale;

			double v_t = 1.0 - 4 * m_lambda * p.x * p.x - 4 * m_lambda * p.y * p.y;
			double u_t = 2.0 * m_lambda * p.x * p.x + 2.0 * m_lambda * p.y * p.y;

			if(v_t >= 0.0 && u_t != 0.0)
			{
				v_t = sqrt(v_t);
				q.x = (p.x - p.x * v_t)/u_t;
				q.y = (p.y - p.y * v_t)/u_t;

				q.x = q.x/m_scale + m_normalizeCenter.x;
				q.y = q.y/m_scale + m_normalizeCenter.y;

				//shift back distortion center
				q.x = q.x + m_imageCenter.x;   //trick part
				q.y = q.y + m_imageCenter.y;

				rem_x[y][x] = q.x;
				rem_y[y][x] = q.y;		

			}
		}
	}
	cv::remap(dImage, sImage, remapX, remapY, INTER_LINEAR, BORDER_CONSTANT);

	//rescale the image	
	cv::resize(sImage, sImage, cv::Size(w, h));	
	return sImage;
}

void CvxRobustHomoImageUndistortion::undistortPoints(vector<Point2f> & inoutPts)
{
	//assert(inoutPts.size() >= 4);

	for(int i = 0; i<inoutPts.size(); i++)
	{
		double x = inoutPts[i].x;
		double y = inoutPts[i].y;

		x = x - m_imageCenter.x;
		y = y - m_imageCenter.y;

		//change to normalized space 
		x = (x - m_normalizeCenter.x) * m_scale;
		y = (y - m_normalizeCenter.y) * m_scale;

		double dis_square = x * x + y * y;
		x = 1.0/(1.0 + m_lambda * dis_square) * x;
		y = 1.0/(1.0 + m_lambda * dis_square) * y;

		//change back to the image space
		x = x/m_scale + m_normalizeCenter.x;
		y = y/m_scale + m_normalizeCenter.y;

		x = x + m_imageCenter.x;
		y = y + m_imageCenter.y;

		//store the result
		inoutPts[i].x = x;
		inoutPts[i].y = y;
	}
}

void CvxRobustHomoImageUndistortion::getNormalizedCenterAndScale(const vector<Point2f> &pts, Point2d &centerPt, double &scale)
{
	assert(pts.size()  >= 2);

	//calculate center point position
	centerPt.x = 0.0;
	centerPt.y = 0.0;
	scale = 1.0;
	for(int i = 0; i < pts.size(); i++)
	{
		centerPt.x += pts[i].x;
		centerPt.y += pts[i].y;
	}
	centerPt.x /= pts.size();
	centerPt.y /= pts.size();

	//scale the average distance to sqrt(2)
	double distance = 0.0;
	for(int i = 0; i< pts.size(); i++)
	{
		double dx = pts[i].x - centerPt.x;
		double dy = pts[i].y - centerPt.y;
		double dis = sqrt(dx * dx + dy * dy);
		distance += dis;
	}

	distance /= pts.size();  //mean distance
	scale = sqrt(2.0)/distance;
}

void CvxRobustHomoImageUndistortion::normalizeData(vector<Point2f> &pts, Point2d &centerPt, double &scale)
{
	assert(pts.size()  >= 2);

	//calculate center point position
	centerPt.x = 0.0;
	centerPt.y = 0.0;
	scale = 1.0;
	for(int i = 0; i < pts.size(); i++)
	{
		centerPt.x += pts[i].x;
		centerPt.y += pts[i].y;
	}
	centerPt.x /= pts.size();
	centerPt.y /= pts.size();

	//scale the average distance to sqrt(2)
	double distance = 0.0;
	for(int i = 0; i< pts.size(); i++)
	{
		double dx = pts[i].x - centerPt.x;
		double dy = pts[i].y - centerPt.y;
		double dis = sqrt(dx * dx + dy * dy);
		distance += dis;
	}

	distance /= pts.size();  //mean distance
	scale = sqrt(2.0)/distance;

	for(int i = 0; i<pts.size(); i++)
	{
		pts[i].x = (pts[i].x - centerPt.x) * scale;
		pts[i].y = (pts[i].y - centerPt.y) * scale;
	}
}