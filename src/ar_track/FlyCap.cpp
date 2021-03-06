#include "FlyCap.h"

using namespace std;

void FlyCap::init()
{

	cout<<"i'm in init in flycap"<<endl;



	error = camera->Connect(0);//open the stream-video




	if ( error != PGRERROR_OK )
   {    std::cout << "Failed to connect to camera" << std::endl; return;  }  
    
		// Get the camera info and print it out
   error = camera->GetCameraInfo( &camInfo );
   if ( error != PGRERROR_OK )
   {    std::cout << "Failed to get camera info from camera" << std::endl;  return;  } 
  
    std::cout << camInfo.vendorName << " "
    		  << camInfo.modelName << " " 
    		  << camInfo.serialNumber << std::endl;
	
/*for setting
	VideoMode *pVideo;
	FrameRate *pFrame;
	camera->GetVideoModeAndFrameRate(pVideo,pFrame);

	cout<<"VideoMode : "<<*pVideo<<" FrameRate : "<<*pFrame<<endl;
	*/



	cout<<"preSTART"<<endl;

	error = camera->StartCapture();
	
	cout<<"postSTART"<<endl;





   if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
   {    std::cout << "Bandwidth exceeded" << std::endl; return;	}
   else if ( error != PGRERROR_OK )
	{ 	std::cout << "Failed to start image capture" << std::endl; return; 	} 
	
}


void FlyCap::flyRaw()
{
		if(camera->IsConnected())
		{	error = camera->RetrieveBuffer( &rawImage );//grab to image
		
		std::cout<<rawImage.GetPixelFormat()<<std::endl;}

		
		if ( error != PGRERROR_OK )
			std::cout << "capture error" << std::endl;
}




void FlyCap::getImageFlyCapRGB8(Image& rgbImage)
{

   rawImage.Convert( FlyCapture2::PIXEL_FORMAT_RGB8, &rgbImage );
 
}


Mat FlyCap::getImageOpencv() // convert to rgb with FlyCap
{
		

		int size= rawImage.GetDataSize();
		cout<<"dimensione in byte di rawImage : "<<size<<endl;

	   Image rgbImage;//type flycap image
		{
   	   rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );
      } 
		
		int ssize = rgbImage.GetDataSize();
		cout<<"dim in byte di rgbImage : "<<ssize<<endl;


		// convert to OpenCV Mat
		unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();       
		image_for_OPENCV = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);

	return image_for_OPENCV;

}


void FlyCap::closeVideo()
{
	error = camera->StopCapture();
    if ( error != PGRERROR_OK )
    {
			// This may fail when the camera was removed, so don't show 
        // an error message
    }  
	
	camera->Disconnect();
}

