#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <arpa/inet.h>
//#include <boost/endian/conversion.hpp>
#include <FlyCap.h>
#include <tf/transform_broadcaster.h>


using namespace std;
using namespace cv;
 
int main(int argc, char** argv)
{
 
	//inizializzazione nodo ROSPTGREY
	ros::init(argc, argv, "FLYCAM_image");// ROS node
   ros::NodeHandle nh;

	//per pubblicare sul topic l'immagine prelevata dalla camera
   image_transport::ImageTransport it(nh);
   image_transport::Publisher pub_img = it.advertise("image", 1);
  
	
	//per pubblicare sul topic le camera_info (matrice intrinseca, etc ) 
	//image_transport::ImageTransport it_f(nh);
	ros::Publisher pub_cam_info = nh.advertise<sensor_msgs::CameraInfo>("camera_info",1);

	// get current CameraInfo data
   sensor_msgs::CameraInfoPtr cam_info = boost::make_shared<sensor_msgs::CameraInfo>();
		


/*
----------------------------------CAMERA CALIBRATION WITH ROS

camera matrix
1185.251619 0.000000 522.728158
0.000000 1184.649020 494.759821
0.000000 0.000000 1.000000

distortion
-0.090211 0.280924 -0.002574 0.002095 0.000000

rectification
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000

projection
1190.698608 0.000000 523.516227 0.000000
0.000000 1190.216064 492.838631 0.000000
0.000000 0.000000 1.000000 0.000000






*/
//----------------// CameraInfo //--------------------



//distortion vector
		cam_info->header.frame_id = "G_optical_frame"; 
      cam_info->height = 1024;
      cam_info->width  = 1024;
		cam_info->distortion_model = "plumb_bob";
		cam_info->D.push_back(-0.090211);
		cam_info->D.push_back(0.280924);
		cam_info->D.push_back(-0.002574);
		cam_info->D.push_back(0.002095);
		cam_info->D.push_back(0.000000);


//camera_matrix
		cam_info->K[0] = 1185.251619;
		cam_info->K[1] = 0.000000;
		cam_info->K[2] = 522.728158;
		cam_info->K[3] = 0.000000;
		cam_info->K[4] = 1184.649020;
		cam_info->K[5] = 494.759821;
		cam_info->K[6] = 0.000000;
		cam_info->K[7] = 0.000000;
		cam_info->K[8] = 1.000000;






//rectification
		cam_info->R[0] = 1.000000;
		cam_info->R[1] = 0.000000;
		cam_info->R[2] = 0.000000;
		cam_info->R[3] = 0.000000;
		cam_info->R[4] = 1.000000;
		cam_info->R[5] = 0.000000;
		cam_info->R[6] = 0.000000;
		cam_info->R[7] = 0.000000;
		cam_info->R[8] = 1.000000;
		



//projetion
		cam_info->P[0] = 1190.698608;
		cam_info->P[1] = 0.000000;
		cam_info->P[2] = 523.516227;
		cam_info->P[3] = 0.000000;
		cam_info->P[4] = 0.000000;
		cam_info->P[5] = 1190.216064;
		cam_info->P[6] = 492.838631;
		cam_info->P[7] = 0.000000;
		cam_info->P[8] = 0.000000;
		cam_info->P[9] = 0.000000;
		cam_info->P[10] = 1.000000;
		cam_info->P[11] = 0.000000;
			




		cam_info->binning_x = 0;
		cam_info->binning_y = 0;
		cam_info->roi.width  = 0;
		cam_info->roi.height = 0;
		cam_info->roi.x_offset = 0;
		cam_info->roi.y_offset = 0;
		cam_info->roi.do_rectify = 0;




		//classe del driver
		FlyCap FCapture;
		FCapture.init();
		Image rgbImg;






		//creiamo un legame fra i sistemi di riferimento
		std::string stringaFrameIdPadre = "link_fisico";
		std::string stringaFrameIdFiglio = "G_optical_frame";


		tf::TransformBroadcaster tf_broadcaster; 



		double px = 0;//p.translation[0]/100.0;
		double py = 0;//p.translation[1]/100.0;
		double pz = 0.03;//metri   //p.translation[2]/100.0;
		double qw = 1;//p.quaternion[0];
		double qx = 0;//p.quaternion[1];
		double qy = 0;//p.quaternion[2];
		double qz = 0;//p.quaternion[3];
		


		tf::Quaternion rotazione(qx,qy,qz,qw);
      	tf::Vector3 traslazione(px,py,pz);
      	tf::Transform trasformazione(rotazione, traslazione);


		
		//tf::StampedTransform camLinkTocamOpt(trasformazione, ros::Time::now(), stringaFrameIdPadre, stringaFrameIdFiglio);
		//tf_broadcaster.sendTransform(camLinkTocamOpt);//pubblicare la trasf su TF(è un pacchetto con un topic)
	



   	ros::Rate loop_rate(100); 

   	while (nh.ok()) 
		{
			



			FCapture.flyRaw(); // acquisizione immagine




   	

			sensor_msgs::ImagePtr img = boost::make_shared<sensor_msgs::Image>(); 
			if(htonl(47)==47){img->is_bigendian = 1; }else{img->is_bigendian = 0;}

			FCapture.getImageFlyCapRGB8(rgbImg);//prelevo l'immagine per inviarla sul topic

			img->header.frame_id =  "G_optical_frame";
			img->height   = rgbImg.GetRows();//source_image.row by direct_flycap;
			img->width    = rgbImg.GetCols();//source_image.col by direct_flycap;
			img->encoding = "rgb8";
			img->step     = rgbImg.GetStride();
			size_t size   = (img->step)*(img->height);
			img->data.resize(size);

			memcpy((char*)(&img->data[0]), rgbImg.GetData(), size);




			// "  TRASFORMAZIONE RIGIDA "
			tf::StampedTransform camLinkTocamOpt(trasformazione, ros::Time::now(), stringaFrameIdPadre, stringaFrameIdFiglio);
			tf_broadcaster.sendTransform(camLinkTocamOpt);



		
			//pubblico sui relativi topic
			pub_img.publish(img);
			pub_cam_info.publish(*cam_info);




	  		ros::spinOnce();
	   	loop_rate.sleep();
	}

	FCapture.closeVideo(); //chiude la camera

}

