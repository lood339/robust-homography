#include "UT_imageUndistortion.h"
#include "cvxImage.h"
#include "cvxRobustHomoImageUndistortion.h"

void UTImageUndistortion::testImageCenterUndistortionSimulateImages()
{
	Mat image = cv::imread("testImage\\simulation\\render_chessboard_0.png", 1);
	
	assert(!image.empty());

	double lambda = -0.003;
	cv::Size patternSize = cv::Size(14, 10);
	cv::imshow("original image", image);
//	cv::imwrite("org.png", image);

	Mat dImage = CvxRobustHomoImageUndistortion::distortImage(image, cv::Size(14, 10), lambda);
	cv::imshow("distorted image", dImage);
//	cv::imwrite("distorted.png", dImage);

	CvxRobustHomoImageUndistortion aICUndistortion;

	bool isOk = aICUndistortion.estimateDistortionParameter(dImage, cv::Size(14, 10), false, 0.0, true);

	if(isOk)
	{
		Mat reverseImage = aICUndistortion.undistortImageWithScale(dImage);
		cv::imshow("reverse image", reverseImage);
	//	cv::imwrite("reversed.png", reverseImage);
	}
	else
	{
		printf("undistortion failed.\n");
		
	}
}

void UTImageUndistortion::testImageCenterUndistortionRealImages()
{
	Mat image = cv::imread("testImage\\zoom_level_0_1.png", 1);
	assert(!image.empty());

	int w = image.cols;
	int h = image.rows;
	cv::resize(image, image, cv::Size(w, h));
	const cv::Size patternSize(14, 10);

	CvxRobustHomoImageUndistortion aICUndistortion;
	bool isOk = aICUndistortion.estimateDistortionParameter(image, patternSize, false, 0.0, true);

	Mat revMat = aICUndistortion.undistortImageWithScale(image);
	cv::imshow("original image", image);
	cv::imshow("undistorted image", revMat);
}

void UTImageUndistortion::testImageCenterUndistortionDoubleImage()
{
	Mat chessboardImage = cv::imread("testImage\\folder_screen\\zoom_level_0_30cm\\left0.png", 1);
	Mat sceneImage      = cv::imread("testImage\\folder_screen\\horse_30\\left0.png", 1);

	assert(!chessboardImage.empty());
	assert(!sceneImage.empty());

	cv::Size patternSize(14, 10);

	CvxRobustHomoImageUndistortion aICUndistortion;

	//calculate distortion parameter
	bool isOk = aICUndistortion.estimateDistortionParameter(chessboardImage, patternSize, false, 0.0, true);

	// undistort the image with a chessboard
	Mat reverseImageA = aICUndistortion.undistortImageWithScale(chessboardImage);

	cv::imshow("original image", chessboardImage);
	cv::imshow("un distored image", reverseImageA);
	cv::imwrite("reference_result.png", reverseImageA);

	// undistort the image without a chessboard but captured by same camera (focal length and focal distance)
	Mat reverseImageB = aICUndistortion.undistortImageWithScale(sceneImage);

	cv::imshow("original scene", sceneImage);
	cv::imshow("un distored scene", reverseImageB);
	cv::imwrite("un_distorted_folding_screen.png", reverseImageB); 
}

void UTImageUndistortion::testCenterUndistortionRecordVideo()
{
	string videoFileName = "testVideo\\zoom_level_0_adjust_depth.avi";

	VideoCapture vc;
	bool isFound = vc.open(videoFileName);
	if(!isFound)
	{
		printf("can not open video file %s\n", videoFileName.c_str());
		return;
	}
	assert(isFound);

	cv::Size patternSize = cv::Size(14, 10);
	
	int num = 0;
	CvxRobustHomoImageUndistortion aICUndistortion;
	aICUndistortion.setBoardSize(patternSize);

	cv::Size mergedFrameSize = cv::Size(960*2, 540);

//	VideoWriter aCvWrite;  //write to a result file
//	aCvWrite.open("un_distorion_compare.avi", CV_FOURCC('M','J','P','G'), 24, mergedFrameSize, true);

	Mat mergedFrame = cv::Mat::zeros(mergedFrameSize.height, mergedFrameSize.width, CV_8UC3); 
	vector<double> unDistortionProcessingTimes;
	for(;;)
	{
		Mat frame;
		vc>>frame;
		if(frame.empty())
		{
			break;
		}	

		bool isOk = aICUndistortion.estimateDistortionParameter(frame, cv::Size(14, 10), true, 0.0, false);

		//undistort image
		Mat uImage = aICUndistortion.undistortImageWithScale(frame);
		

		int h = frame.rows/2;
		int w = frame.cols/2;
	
		num++;
		cv::resize(frame, frame, cv::Size(w, h));		
		cv::resize(uImage, uImage, cv::Size(w, h));
		
		frame.copyTo(mergedFrame(Rect(0, 0, 960, 540)));
		uImage.copyTo(mergedFrame(Rect(960, 0, 960, 540)));
		cv::imshow("undistortion", mergedFrame);
		printf("done one frame.\n"); 
		 
		

		if(num > 60 && num < 22 * 24)
		{
			cv::imshow("merged frame", mergedFrame);
		//	aCvWrite<<mergedFrame;
		}		

		cvWaitKey(10);
	}
}