#ifndef UT_IMAGE_UNDISTORTION_H
#define UT_IMAGE_UNDISTORTION_H

class UTImageUndistortion
{
public:
	// test simulate image
	static void testImageCenterUndistortionSimulateImages();
	// test on real image
	static void testImageCenterUndistortionRealImages();

	// undistort the image by a reference image
	static void testImageCenterUndistortionDoubleImage();
	
	// undistort images in a video
	static void testCenterUndistortionRecordVideo();



};


#endif