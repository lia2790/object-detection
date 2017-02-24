#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <arpa/inet.h>


#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>



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


//______________BOARD IN THE SYS________________________________________
//
//_______________________________________________________________________
//_______________________________________________________________________




std::string board1_to_object_frame = "/board1x8";


CPose3D board1(0, 0, -0.036, DEG2RAD(0.0), DEG2RAD(0.0), DEG2RAD(0.0) );

CPose3DQuat board1_ = CPose3DQuat(board1);


		double tx1 = board1_.m_coords[0];	//  array_traslation t=tx,ty,tz
		double ty1 = board1_.m_coords[1];
		double tz1 = board1_.m_coords[2];
		double qw1 = board1_.m_quat[0];		// array_quaterion
		double qx1 = board1_.m_quat[1];
		double qy1 = board1_.m_quat[2];
		double qz1 = board1_.m_quat[3];	


tf::Vector3 trasl1(tx1, ty1, tz1);
tf::Quaternion rot1(qx1, qy1, qz1, qw1);
tf::Transform trasformazione_B1toObj(rot1, trasl1);





std::string board2_to_object_frame = "/board2x8";


CPose3D board2(0, 0, -0.036, DEG2RAD(0.0), DEG2RAD(0.0), DEG2RAD(-72.0) ); // x y z - yaw_z pitch_y roll_x

CPose3DQuat board2_ = CPose3DQuat(board2);


		double tx2 = board2_.m_coords[0];	//  array_traslation t=tx,ty,tz
		double ty2 = board2_.m_coords[1];
		double tz2 = board2_.m_coords[2];
		double qw2 = board2_.m_quat[0];		// array_quaterion
		double qx2 = board2_.m_quat[1];
		double qy2 = board2_.m_quat[2];
		double qz2 = board2_.m_quat[3];	


tf::Vector3 trasl2(tx2, ty2, tz2);
tf::Quaternion rot2(qx2, qy2, qz2, qw2);
tf::Transform trasformazione_B2toObj(rot2, trasl2);



std::string board3_to_object_frame = "/board3x8";

CPose3D board3(0, 0, -0.036, DEG2RAD(0.0), DEG2RAD(0.0), DEG2RAD(-144.0) );

CPose3DQuat board3_ = CPose3DQuat(board3);


		double tx3 = board3_.m_coords[0];	//  array_traslation t=tx,ty,tz
		double ty3 = board3_.m_coords[1];
		double tz3 = board3_.m_coords[2];
		double qw3 = board3_.m_quat[0];		// array_quaterion
		double qx3 = board3_.m_quat[1];
		double qy3 = board3_.m_quat[2];
		double qz3 = board3_.m_quat[3];	


tf::Vector3 trasl3(tx3, ty3, tz3);
tf::Quaternion rot3(qx3, qy3, qz3, qw3);
tf::Transform trasformazione_B3toObj(rot3, trasl3);




std::string board4_to_object_frame = "/board4x8";

CPose3D board4(0, 0, -0.036, DEG2RAD(0.0), DEG2RAD(0.0), DEG2RAD(-216.0) );

CPose3DQuat board4_ = CPose3DQuat(board4);


		double tx4 = board4_.m_coords[0];	//  array_traslation t=tx,ty,tz
		double ty4 = board4_.m_coords[1];
		double tz4 = board4_.m_coords[2];
		double qw4 = board4_.m_quat[0];		// array_quaterion
		double qx4 = board4_.m_quat[1];
		double qy4 = board4_.m_quat[2];
		double qz4 = board4_.m_quat[3];	


tf::Vector3 trasl4(tx4, ty4, tz4);
tf::Quaternion rot4(qx4, qy4, qz4, qw4);
tf::Transform trasformazione_B4toObj(rot4, trasl4);





std::string board5_to_object_frame = "/board5x8";

CPose3D board5(0, 0, -0.036, DEG2RAD(0.0), DEG2RAD(0.0), DEG2RAD(-288.0) );

CPose3DQuat board5_ = CPose3DQuat(board5);


		double tx5 = board5_.m_coords[0];	//  array_traslation t=tx,ty,tz
		double ty5 = board5_.m_coords[1];
		double tz5 = board5_.m_coords[2];
		double qw5 = board5_.m_quat[0];		// array_quaterion
		double qx5 = board5_.m_quat[1];
		double qy5 = board5_.m_quat[2];
		double qz5 = board5_.m_quat[3];	


tf::Vector3 trasl5(tx5, ty5, tz5);
tf::Quaternion rot5(qx5, qy5, qz5, qw5);
tf::Transform trasformazione_B5toObj(rot5, trasl5);



//risultato----->POSE_COMPOSITION
		double tx_pc = 0;		// array_traslation t=tx,ty,tz
		double ty_pc = 0;
		double tz_pc = 0;
		double qw_pc = 1;		// array_quaterion			
		double qx_pc = 0;
		double qy_pc = 0;
		double qz_pc = 0;
//inizializzazione di default: nessuna traslazione e nessuna rotazione
//_______________________________________________________________________
//_______________________________________________________________________
//_______________________________________________________________________
//_______________________________________________________________________



	ros::Publisher pose_obj_pub;			// Pose with reference coordinate frame and timestamp
	ros::Publisher position_obj_pub;		// It is only meant to represent a direction
	ros::Publisher transform_obj_pub;	// This expresses a transform from coordinate frame header.frame_id to the coordinate frame child_frame_id

//_______________________________________________________________________
//_______________________________________________________________________
//_______________________________________________________________________
//_______________________________________________________________________



void pose_composition_callback(const geometry_msgs::TransformStamped::ConstPtr& Transform_estimate)
{	

	//trasformazione da G_optical_frame a board
	double tx_r = 0;
	double ty_r = 0;
	double tz_r = 0;
	double qw_r = 1;					
	double qx_r = 0;
	double qy_r = 0;
	double qz_r = 0;
	

	tf::Transform rigid_transform;

	static tf::TransformBroadcaster br;

	//elementi della matrice di rotazione
	double r1_r = 0;
	double r2_r = 0;
	double r3_r = 0;
	double r4_r = 0;					
	double r5_r = 0;
	double r6_r = 0;
	double r7_r = 0;
	double r8_r = 0;
	double r9_r = 0;


	//rappresentazione 7D della pos+orient prelevati dal topic
	double tx_e = Transform_estimate->transform.translation.x;
	double ty_e = Transform_estimate->transform.translation.y;
	double tz_e = Transform_estimate->transform.translation.z;

	double qw_e = Transform_estimate->transform.rotation.w;
	double qx_e = Transform_estimate->transform.rotation.x;
	double qy_e = Transform_estimate->transform.rotation.y;
	double qz_e = Transform_estimate->transform.rotation.z;


	//creo la trasformazione stimata dalla visione
	tf::Vector3 trasl(tx_e, ty_e, tz_e);
	tf::Quaternion rot(qx_e, qy_e, qz_e, qw_e);
	tf::Transform estimate_transform(rot, trasl);

	
//a seconda della board riconosciuta dall'image processing effettuo la trasformazione rigida relativa
if(Transform_estimate->child_frame_id == "/board1x8") {	
																				tx_r = tx1;
																				ty_r = ty1;
																				tz_r = tz1;
																				qw_r = qw1;					
																				qx_r = qx1;
																				qy_r = qy1;
																				qz_r = qz1;

																				rigid_transform = trasformazione_B1toObj;
																				
																			}
			
if(Transform_estimate->child_frame_id == "/board2x8") {	
																				tx_r = tx2;
																				ty_r = ty2;
																				tz_r = tz2;
																				qw_r = qw2;					
																				qx_r = qx2;
																				qy_r = qy2;
																				qz_r = qz2;

																				rigid_transform = trasformazione_B2toObj;
																				

																			}
						
if(Transform_estimate->child_frame_id == "/board3x8") {	
																				tx_r = tx3;
																				ty_r = ty3;
																				tz_r = tz3;
																				qw_r = qw3;					
																				qx_r = qx3;
																				qy_r = qy3;
																				qz_r = qz3;

																				rigid_transform = trasformazione_B3toObj;
																				
																			}


if(Transform_estimate->child_frame_id == "/board4x8") {	
																				tx_r = tx4;
																				ty_r = ty4;
																				tz_r = tz4;
																				qw_r = qw4;					
																				qx_r = qx4;
																				qy_r = qy4;
																				qz_r = qz4;

																				rigid_transform = trasformazione_B4toObj;
															
																			}

if(Transform_estimate->child_frame_id == "/board5x8") {	
																				tx_r = tx5;
																				ty_r = ty5;
																				tz_r = tz5;
																				qw_r = qw5;					
																				qx_r = qx5;
																				qy_r = qy5;
																				qz_r = qz5;

																				rigid_transform = trasformazione_B5toObj;
																				
																			}


//________________POSE_COMPOSITION_______________________________
//_______________________________________________________________
//_______________________________________________________________


	//POSE_COMPOSITION_WITH_MRPT

	//calcolo delle pose tramite la libreria MRPT
	CPose3DQuat p_e( tx_e, ty_e, tz_e, CQuaternionDouble( qw_e, qx_e, qy_e, qz_e));	 // trasformazione stimata
	CPose3DQuat p_r( tx_r, ty_r, tz_r, CQuaternionDouble( qw_r, qx_r, qy_r, qz_r));   // tx, ty, tz, qw, qx, qy, qz
	CPose3DQuat pc;
	
	pc = p_e + p_r ; 				// easy		




	
	cout<<"transform-------------->  POSE_COMPOSITION : "<<endl;
	cout<<"Vector_translation : tx "<<pc.m_coords[0]<<" ty "<<pc.m_coords[1]<<" tz "<<pc.m_coords[2]<<endl;
	cout<<"quaternion_orientation : qw "<<pc.m_quat[0]<<" qx "<<pc.m_quat[1]<<" qy "<<pc.m_quat[2]<<" qz "<<pc.m_quat[3]<<endl;
	cout<<"------------------------"<<endl;



	//create a transform with mrpt
	tf::Vector3 traslazione_objMRPT(pc.m_coords[0], pc.m_coords[1], pc.m_coords[2]);					//  x  y  z
	tf::Quaternion rotazione_objMRPT(pc.m_quat[1], pc.m_quat[2], pc.m_quat[3], pc.m_quat[0]);    // qx qy qz qw
	tf::Transform trasformazione_G_opt_To_ObjMRPT(rotazione_objMRPT, traslazione_objMRPT);
	tf::StampedTransform stampedTransformMRPT(trasformazione_G_opt_To_ObjMRPT, ros::Time::now(), Transform_estimate->header.frame_id, "/object_frame_MRPT");


	br.sendTransform(stampedTransformMRPT);	//BROADCAST TF 

	
	// SEND_ON_TOPIC_OUTPUT-------->	trasf_Cam_To_CENTER_Obj

			//PUBLISH /pose      
			geometry_msgs::PoseStamped poseMsg;
			tf::poseTFToMsg(trasformazione_G_opt_To_ObjMRPT, poseMsg.pose);
			poseMsg.header.frame_id = "G_optical_frame"; //padre
			poseMsg.header.stamp    = Transform_estimate->header.stamp;
			pose_obj_pub.publish(poseMsg);


			//PUBLISH /transform
			geometry_msgs::TransformStamped transformMsg;
			tf::transformStampedTFToMsg(stampedTransformMRPT, transformMsg);     //------>  Kalman prende in ingresso questa misura
			transform_obj_pub.publish(transformMsg);


			//PUBLISH position
			geometry_msgs::Vector3Stamped positionMsg;
			positionMsg.header = transformMsg.header;
			positionMsg.vector = transformMsg.transform.translation;
			position_obj_pub.publish(positionMsg);


}




//////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////



int main(int argc, char** argv)
{
 
	//inizializzazione nodo 
	ros::init(argc, argv, "Object_geometry");// ROS node
   ros::NodeHandle nn;


	cout<<"1"<<endl;

	// pub
	pose_obj_pub = nn.advertise<geometry_msgs::PoseStamped>("/pose_obj", 1);
	position_obj_pub = nn.advertise<geometry_msgs::Vector3Stamped>("/position_obj", 1);
	transform_obj_pub = nn.advertise<geometry_msgs::TransformStamped>("/transform_obj", 1);

	
	// sub 
	ros::Subscriber pose_board_sub;
	pose_board_sub = nn.subscribe("/ar_multi_boards/transform", 1, &pose_composition_callback);



	ros::Rate loop_rate(100);


	while(nn.ok())
	{	
		
		ros::spinOnce();
		loop_rate.sleep();
		
	}

}
  

