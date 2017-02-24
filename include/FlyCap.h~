#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "FlyCapture2.h"
#include <iostream>
#include <thread>
#include <mutex>

using namespace FlyCapture2;
using namespace cv;
using namespace std;

class FlyCap	
{
	public:

	FlyCapture2::Error error;
	Camera *camera;//typeFLYCAP
	CameraInfo camInfo;//typeFLyCAP
	Image rawImage;//typeFLYCAP
	Mat image_for_OPENCV;

	FlyCap() {camera = new Camera; }
	~FlyCap() { error = camera->StopCapture();
    				if ( error != PGRERROR_OK ){}
						camera->Disconnect();
					delete camera;}

	/*void startVideo();//open camera and start to capture the frame
	void takeImageFromVideo();//convert a image from FlyCap to OpenCV
	void closeVideo();//close video and camera
	Mat getImageOpencv() {	return image_for_OPENCV; } 
	*/

	void init();
	void flyRaw();
	void getImageFlyCapRGB8(Image& rgbImage);

	//conversione
	Mat getImageOpencv();
	//??? getImageROSimageTRANSPORT();

	void closeVideo();
	
};
