#include "cvxImage.h"
#include "markerchessboard.hpp"
#include "UT_imageUndistortion.h"

#if 1
int main()
{
	UTImageUndistortion::testImageCenterUndistortionSimulateImages();
	UTImageUndistortion::testImageCenterUndistortionRealImages();
	UTImageUndistortion::testImageCenterUndistortionDoubleImage();
	UTImageUndistortion::testCenterUndistortionRecordVideo();
	

	cvWaitKey(0);

	return 0;
}
#endif

