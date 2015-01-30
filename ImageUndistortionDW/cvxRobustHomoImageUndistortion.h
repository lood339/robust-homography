#ifndef CVX_ROBUST_HOMO_IMAGE_UNDISTORTION_H
#define CVX_ROBUST_HOMO_IMAGE_UNDISTORTION_H 1

/*
  This is an implementation of "Robust homography for real-time image un-distortion"
  As sample code in engage project.
*/

#include "cvxImage.h"
#include "markerchessboard.hpp"

class CvxRobustHomoImageUndistortion
{
public:
	CvxRobustHomoImageUndistortion();
	~CvxRobustHomoImageUndistortion();

	/*
		patternSize: chessboard pattern size, 14 x 10
	*/	
	void setBoardSize(const cv::Size &patternSize);

	/* 
	  calculate undistortion parameter
	  inputImage: an image of chessboard
	  patternSize: chessboard pattern size, like 14 x 10
	  isTracking:  
	               true: video
				   false: single image
	  noiseStdDevToCorners: 0.0, only unsed in test
	  showResult:           false, only used in test
	*/

	bool estimateDistortionParameter(const Mat &inputImage, const cv::Size &patternSize, bool isTracking, double noiseStdDevToCorners, bool showResult);	


	/* un-distort points according to the parameters
	   make sure that estimateDistortionParameter is called before calling this function
    */
	void undistortPoints(vector<Point2f> & inoutPts);


	/* undistort an image according to calculated lambda, some boundary areas will be outside of the image
	   distortedImage: input image
	   return        : undistorted image	   
	*/
	Mat undistortImage(const Mat &distortedImage);


	/* undistort an image according to calculated lambda with a scale so that all original pixel will be inside of the image
	   distortedImage: input image
	   return        : undistorted image	   
	*/
	Mat undistortImageWithScale(const Mat &distortedImage);

	
	double lambda()  {return m_lambda;}
	Point2d normalizeCenter() {return m_normalizeCenter;}
	Point2d distortionCenter() {return m_imageCenter;}
	double  scale()  {return m_scale;}

private:
	double  m_lambda;             //distortion parameter lambda in the paper
	Point2d m_imageCenter;        //image center in image space

	Point2d m_normalizeCenter;    //center point location for normalized data
	double  m_scale;              //scale corner locations to normalized space

	CvMarkerChessboard *pBoard;   //chessboard for corner detection/tracking

private:
	/*
		calculate undistortion parameter from normalized corners
		boardCorners: normalized board corners
		imageCorners: normalized image corners

	*/
	bool estimateDistortionParameter(const vector<Point2f> &boardCorners, const vector<Point2f> &imageCorners);

	//solve equation (3)
	static bool solveOnce(const vector<double> &residence, const vector<Point2f> &boardPts, 
		                  const vector<Point2f> &corners, vector<double> &result);

public:
	//distort an image. This function is used to generate distortion image for test purpose
	static Mat distortImage(const Mat &orgImage, const cv::Size &patternSize, double lambda);

	// get normalized center and scale for given points
	static void getNormalizedCenterAndScale(const vector<Point2f> &pts, Point2d &centerPt, double &scale);

	//normalize corner positions in pts and get normalized center point and scale
	static void normalizeData(vector<Point2f> &pts, Point2d &centerPt, double &scale);

	

};

#endif
