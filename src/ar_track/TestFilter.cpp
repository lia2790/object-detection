
#include <iostream>
#include <fstream>
#include <sstream>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <cmath>


#include <mrpt/base.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose3DQuat.h>



using namespace std;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::utils;



ros::Publisher pose_obj_pub;			
ros::Publisher position_obj_pub;		
ros::Publisher transform_obj_pub;




int main(int argc, char** argv)
{
	ros::init(argc, argv, "TestKalman");		// ROS node
   ros::NodeHandle nn;

   // pub
	pose_obj_pub = nn.advertise<geometry_msgs::PoseStamped>("/pose_obj", 1);
	position_obj_pub = nn.advertise<geometry_msgs::Vector3Stamped>("/position_obj", 1);
	transform_obj_pub = nn.advertise<geometry_msgs::TransformStamped>("/transform_obj", 1);



	static tf::TransformBroadcaster br;




	ros::Rate loop_rate(100);

	
bool run = true;

double incremento = 0.0005;
double start = 0.30;
double count = 0.0;


double incremento_angle = 1.0;
double start_angle = 0.0;
double count_angle = 0.0;


	while(nn.ok())
	{
		if(run)	
		{
		 	//cout<<"run true "<<run<<endl;

			cout<<"INC_pos : "<< (start+count) << "" << (start_angle+count_angle) <<endl;
			
			CPose3D board1((start+count), (start+count), (start+count), DEG2RAD(0.0), DEG2RAD(0.0), DEG2RAD(start_angle+count_angle) );

			CPose3DQuat board1_ = CPose3DQuat(board1);


		double tx1 = board1_.m_coords[0];	//  array_traslation t=tx,ty,tz
		double ty1 = board1_.m_coords[1];
		double tz1 = board1_.m_coords[2];
		double qw1 = board1_.m_quat[0];		// array_quaterion
		double qx1 = board1_.m_quat[1];
		double qy1 = board1_.m_quat[2];
		double qz1 = board1_.m_quat[3];	


    tf::Vector3 traslazione(tx1, ty1, tz1);
    tf::Quaternion rotazione(qx1, qy1, qz1, qw1);
//		tf::Transform trasformazione_B1toObj(rot1, trasl1);


//			tf::Vector3 traslazione((0.30+count), (0.30+count), (0.30+count));		       //  x  y  z
//			tf::Quaternion rotazione(0.5, 0.5, 0.5, 0.5);    // qx qy qz qw
			tf::Transform trasformazione(rotazione, traslazione);
			tf::StampedTransform stamped_Transform(trasformazione, ros::Time::now(), "G_optical_frame", "/Reference");
		
			br.sendTransform(stamped_Transform);	//BROADCAST TF ------>  RVIZ




			count += incremento;
			count_angle += incremento_angle;	
			





				//PUBLISH /pose      
			geometry_msgs::PoseStamped poseMsg;
			tf::poseTFToMsg(trasformazione, poseMsg.pose);
			poseMsg.header.frame_id = "G_optical_frame"; //padre
			poseMsg.header.stamp    = ros::Time::now();
			pose_obj_pub.publish(poseMsg);


			//PUBLISH /transform
			geometry_msgs::TransformStamped transformMsg;
			tf::transformStampedTFToMsg(stamped_Transform, transformMsg);     //------>  Kalman prende in ingresso questa misura
			transform_obj_pub.publish(transformMsg);


			//PUBLISH position
			geometry_msgs::Vector3Stamped positionMsg;
			positionMsg.header = transformMsg.header;
			positionMsg.vector = transformMsg.transform.translation;
			position_obj_pub.publish(positionMsg);



		}
		else
		{
			cout<<"run false "<<run<<endl;

			tf::Vector3 traslazione(0.30, 0.30, 0.30);		       //  x  y  z
			tf::Quaternion rotazione(0.5, 0.5, 0.5, 0.5);    // qx qy qz qw
			tf::Transform trasformazione(rotazione, traslazione);
			tf::StampedTransform stamped_Transform(trasformazione, ros::Time::now(), "G_optical_frame", "/Reference");
		
			br.sendTransform(stamped_Transform);	//BROADCAST TF ------>  RVIZ



				//PUBLISH /pose      
			geometry_msgs::PoseStamped poseMsg;
			tf::poseTFToMsg(trasformazione, poseMsg.pose);
			poseMsg.header.frame_id = "G_optical_frame"; //padre
			poseMsg.header.stamp    = ros::Time::now();
			pose_obj_pub.publish(poseMsg);


			//PUBLISH /transform
			geometry_msgs::TransformStamped transformMsg;
			tf::transformStampedTFToMsg(stamped_Transform, transformMsg);     //------>  Kalman prende in ingresso questa misura
			transform_obj_pub.publish(transformMsg);


			//PUBLISH position
			geometry_msgs::Vector3Stamped positionMsg;
			positionMsg.header = transformMsg.header;
			positionMsg.vector = transformMsg.transform.translation;
			position_obj_pub.publish(positionMsg);

		}

//		br.sendTransform(stamped_Transform);	//BROADCAST TF ------>  RVIZ




		


		loop_rate.sleep();
	}


}
