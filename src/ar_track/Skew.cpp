#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <arpa/inet.h>
//#include <boost/endian/conversion.hpp>
#include <tf/transform_broadcaster.h>


#include <nav_msgs/Odometry.h>


#include <unsupported/Eigen/MatrixFunctions>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>


#include <cmath>



using namespace std;
using namespace cv;
using namespace Eigen;



double dt = 1;

Eigen::VectorXd		p_k(3);
Eigen::VectorXd 		q_k(4);
Eigen::VectorXd		q_k_1(4);
Eigen::VectorXd		q_k_dot(4);
Eigen::MatrixXd 		R_k(3,3);		//passo attuale
Eigen::MatrixXd 		R_k_1(3,3);		//passo precedente
Eigen::MatrixXd 		R_k_dot(3,3);
Eigen::MatrixXd 		R_k_dot2(3,3);
Eigen::MatrixXd		S_w(3,3);
Eigen::MatrixXd		S_w2(3,3);
Eigen::MatrixXd		E_q(4,3);
Eigen::VectorXd		w(3);




void skew(const geometry_msgs::TransformStamped::ConstPtr& Transform_estimate)
{


std::cout<<"---------------FRAME_-----------------------"<< endl;
	cout <<	Transform_estimate->header.frame_id << endl;
std::cout<<"--------------------------------------"<< endl;



std::cout<<"---------------FRAME_CHILD-----------------------"<< endl;
	cout <<	Transform_estimate->child_frame_id <<endl;
std::cout<<"--------------------------------------"<< endl;

//____prendo la misura corrispondente al centro dell'oggetto

		p_k[0] = Transform_estimate->transform.translation.x;
		p_k[1] = Transform_estimate->transform.translation.y;
		p_k[2] = Transform_estimate->transform.translation.z;
		q_k[0] = Transform_estimate->transform.rotation.w; 			
		q_k[1] = Transform_estimate->transform.rotation.x;			
		q_k[2] = Transform_estimate->transform.rotation.y;		
		q_k[3] = Transform_estimate->transform.rotation.z;			



//		q[0] w q[1] x  q[2] y  q[3] z


		std::cout<<"---------------QUATERNION-----------------------"<< endl;
		std::cout<<  q_k[0] << endl;
		std::cout<<  q_k[1] << endl;
		std::cout<<  q_k[2] << endl;
		std::cout<<  q_k[3] << endl;
		std::cout<<"------------------------------------------------"<< endl;



		std::cout<<"---------------QUATERNION_PREVIOUS--------------"<< endl;
		std::cout<<  q_k_1[0] << endl;
		std::cout<<  q_k_1[1] << endl;
		std::cout<<  q_k_1[2] << endl;
		std::cout<<  q_k_1[3] << endl;
		std::cout<<"------------------------------------------------"<< endl;



		std::cout<<"---------------SAMPLING_TIME--------------------"<< endl;
		std::cout<<  dt << endl;
		std::cout<<"------------------------------------------------"<< endl;


		

		q_k_dot(0) = ( q_k(0) - q_k_1(0) ) / dt ;   
		q_k_dot(1) = ( q_k(1) - q_k_1(1) ) / dt ;   
		q_k_dot(2) = ( q_k(2) - q_k_1(2) ) / dt ;  
		q_k_dot(3) = ( q_k(3) - q_k_1(3) ) / dt ;  

	
		std::cout<<"---------------QUATERNION_DOT-------------------"<< endl;
		std::cout<<  q_k_dot[0] << endl;
		std::cout<<  q_k_dot[1] << endl;
		std::cout<<  q_k_dot[2] << endl;
		std::cout<<  q_k_dot[3] << endl;
		std::cout<<"------------------------------------------------"<< endl;

		


	

		E_q(0,0) = - q_k(1) ;
		E_q(0,1) = - q_k(2) ;
		E_q(0,2) = - q_k(3) ;
		E_q(1,0) =   q_k(0) ;
		E_q(1,1) =   q_k(3) ;
		E_q(1,2) = - q_k(2) ;
		E_q(2,0) = - q_k(3) ;
		E_q(2,1) =   q_k(0) ;
		E_q(2,2) =   q_k(1) ;
		E_q(3,0) =   q_k(2) ;
		E_q(3,1) = - q_k(1) ;
		E_q(3,2) =   q_k(0) ;


		
		std::cout<<"---------------E_q-----------------------------"<< endl;
		std::cout<<  E_q << endl;
		std::cout<<"------------------------------------------------"<< endl;



	
		w = 2*E_q.transpose()*q_k_dot ;



		std::cout<<"---------------ANGULAR_VELOCITY--------------------"<< endl;
		std::cout<<  w << endl;
		std::cout<<"------------------------------------------------"<< endl;






		R_k(0,0)	= q_k[0]*q_k[0] + q_k[1]*q_k[1] - q_k[2]*q_k[2] - q_k[3]*q_k[3];
		R_k(0,1)	= 2*q_k[1]*q_k[2] - 2*q_k[0]*q_k[3];
		R_k(0,2)	= 2*q_k[1]*q_k[3] + 2*q_k[0]*q_k[2];
		R_k(1,0)	= 2*q_k[1]*q_k[2] + 2*q_k[0]*q_k[3];
		R_k(1,1)	= q_k[0]*q_k[0] + q_k[2]*q_k[2] - q_k[1]*q_k[1] - q_k[3]*q_k[3];
		R_k(1,2)	= 2*q_k[2]*q_k[3] - 2*q_k[0]*q_k[1];
		R_k(2,0)	= 2*q_k[1]*q_k[3] - 2*q_k[0]*q_k[2];
		R_k(2,1)	= 2*q_k[2]*q_k[3] + 2*q_k[0]*q_k[1];
		R_k(2,2)	= q_k[0]*q_k[0] + q_k[3]*q_k[3] - q_k[2]*q_k[2] - q_k[1]*q_k[1];

		

		//			R_k_dot = ( R_k - R_k_1 ) / dt ;	


		R_k_dot(0,0)	= ( R_k(0,0) - R_k_1(0,0) ) / dt ;
		R_k_dot(0,1)	= ( R_k(0,1) - R_k_1(0,1) ) / dt ;
		R_k_dot(0,2)	= ( R_k(0,2) - R_k_1(0,2) ) / dt ;
		R_k_dot(1,0)	= ( R_k(1,0) - R_k_1(1,0) ) / dt ;
		R_k_dot(1,1)	= ( R_k(1,1) - R_k_1(1,1) ) / dt ;
		R_k_dot(1,2)	= ( R_k(1,2) - R_k_1(1,2) ) / dt ;
		R_k_dot(2,0)	= ( R_k(2,0) - R_k_1(2,0) ) / dt ;
		R_k_dot(2,1)	= ( R_k(2,1) - R_k_1(2,1) ) / dt ;
		R_k_dot(2,2)	= ( R_k(2,2) - R_k_1(2,2) ) / dt ;


		
		R_k_dot2(0,0)	= 2*(q_k(0)*q_k_dot(0) + q_k(1)*q_k_dot(1) - q_k(2)*q_k_dot(2) - q_k(3)*q_k_dot(3));
		R_k_dot2(0,1)	= 2*(q_k_dot(1)*q_k(2) + q_k(1)*q_k_dot(2) - q_k_dot(0)*q_k(3) - q_k(0)*q_k_dot(3));
		R_k_dot2(0,2)	= 2*(q_k_dot(1)*q_k(3) + q_k(1)*q_k_dot(3) + q_k_dot(0)*q_k(2) + q_k(0)*q_k_dot(2));
		R_k_dot2(1,0)	= 2*(q_k_dot(1)*q_k(2) + q_k(1)*q_k_dot(2) + q_k_dot(0)*q_k(3) + q_k(0)*q_k_dot(3));
		R_k_dot2(1,1)	= 2*(q_k(0)*q_k_dot(0) - q_k(1)*q_k_dot(1) + q_k(2)*q_k_dot(2) - q_k(3)*q_k_dot(3));
		R_k_dot2(1,2)	= 2*(q_k_dot(2)*q_k(3) + q_k(2)*q_k_dot(3) - q_k_dot(0)*q_k(1) - q_k(0)*q_k_dot(1));
		R_k_dot2(2,0)	= 2*(q_k_dot(1)*q_k(3) + q_k(1)*q_k_dot(3) - q_k_dot(0)*q_k(2) - q_k(0)*q_k_dot(2));
		R_k_dot2(2,1)	= 2*(q_k_dot(2)*q_k(3) + q_k(2)*q_k_dot(3) + q_k_dot(0)*q_k(1) + q_k(0)*q_k_dot(1));
		R_k_dot2(2,2)	= 2*(q_k(0)*q_k_dot(0) + q_k(1)*q_k_dot(1) - q_k(2)*q_k_dot(2) - q_k(3)*q_k_dot(3));

	

		S_w2 = R_k_dot2 * R_k.transpose() ;

		S_w = R_k_dot * R_k.transpose() ;			//body_frame
		

/*
	
		std::cout<<"---------------SAMPLING_TIME--------------------"<< endl;
		std::cout<<  dt << endl;
		std::cout<<"------------------------------------------------"<< endl;

*/
		

		std::cout<<"----------------ROTATION_MATRIX-----------------"<< endl;
		std::cout<<  R_k << endl;
		std::cout<<"------------------------------------------------"<< endl;





   	std::cout<<"-------------DOT_ROTATION_MATRIX----------------"<< endl;
		std::cout<<  R_k_dot << endl;
		std::cout<<"------------------------------------------------"<< endl;



		std::cout<<"-----------------SKEW_MATRIX--------------------"<< endl;
		std::cout<<  S_w << endl;
		std::cout<<"------------------------------------------------"<< endl;


/*

	   std::cout<<"----------2_____________DOT_ROTATION_MATRIX----------------"<< endl;
		std::cout<<  R_k_dot2 << endl;
		std::cout<<"------------------------------------------------"<< endl;




		std::cout<<"----------2_______SKEW_MATRIX--------------------"<< endl;
		std::cout<<  S_w2 << endl;
		std::cout<<"------------------------------------------------"<< endl;


*/

		R_k_1 = R_k ;

		q_k_1 = q_k ;

}




int main(int argc, char** argv)
{
 
	//inizializzazione nodo 
	ros::init(argc, argv, "skew");// ROS node
   ros::NodeHandle nn;


	cout<<"1"<<endl;

	
	
	ros::Subscriber pose_sub;
	pose_sub = nn.subscribe("/transform_obj", 1, &skew);



	ros::Rate loop_rate(100);//RATE, non metterlo al massimo sennò occupo tutta la CPU ( no good )

	cout<<"2"<<endl;



//inizializzazione 


	p_k     <<  0,0,0;
	q_k	  <<  1,0,0,0;
	q_k_1	  <<  1,0,0,0;
	q_k_dot <<  1,0,0,0;


	w			<< 0,0,0 ;


	R_k     <<  MatrixXd::Zero(3,3);	
	R_k_1   <<  MatrixXd::Zero(3,3);	
	R_k_dot <<  MatrixXd::Zero(3,3);	
	R_k_dot2<<  MatrixXd::Zero(3,3); 
	E_q     <<  MatrixXd::Zero(4,3);	







	
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();


	while(nn.ok())
	{

		current_time = ros::Time::now();
		dt = (current_time - last_time).toSec();
	
		
		ros::spinOnce();

  		
		last_time = current_time;

		loop_rate.sleep();
		
	}

}
  

