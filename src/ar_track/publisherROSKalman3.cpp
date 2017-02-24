#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <unsupported/Eigen/MatrixFunctions>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>


#include <cmath>



using namespace std;
using namespace Eigen;



ofstream file_output_comparison;


ros::Publisher pose_KALMAN_pub;		 // Pose with reference coordinate frame and timestamp
ros::Publisher position_KALMAN_pub;	 // It is only meant to represent a direction
ros::Publisher transform_KALMAN_pub; // This expresses a transform from coordinate frame header.frame_id to the coordinate frame child_frame_id



//________tuning_Filter_from_Param_file________


  std::vector<double> P_filter;
  std::vector<double> Q_process;
  std::vector<double> R_sensor;
  std::string file_name;





//____________FILTER PARAMETERS_________________


	int dim_state = 14;
	int dim_measure = 7;

	double dt = 1;

//variazione sulla posizione
	double v_x = 0;
	double v_y = 0;
	double v_z = 0;

//variazione sull'orientazione
	double v_qw = 0;
	double v_qx = 0;
	double v_qy = 0;
	double v_qz = 0;


	//_________state_vector_7D_pose[ x, y, z, qr, qx, qy, qz ]
	//

	//INITIALIZATION

//Matrix of state

	Eigen::MatrixXd F_k(dim_state,dim_state);

	

//matrix of output

	Eigen::MatrixXd H_k(dim_measure,dim_state);

	

//covariance_matrix_of_state

	Eigen::MatrixXd P_k(dim_state,dim_state);

	


//covariance_matrix_model_error

	Eigen::MatrixXd Q_k(dim_state,dim_state);

	


//covariance_matrix_sensor_noise

	Eigen::MatrixXd R_k(dim_measure,dim_measure);



	
	Eigen::MatrixXd I(dim_state,dim_state);







//_________state of KALMAN_FILTER_____________-


	
	//____________PREDICTION_________

	Eigen::VectorXd 		X_k1_k(dim_state);
								
	Eigen::MatrixXd 		P_k1_k(dim_state,dim_state);





	//__________KALMAN_GAIN__________

	
	Eigen::VectorXd 		Y_k1(dim_measure);		
								
	Eigen::MatrixXd 		KALMAN(dim_state,dim_measure);





	//___________UPDATE______________

	Eigen::VectorXd 		X_k1_k1(dim_state);
								
	Eigen::MatrixXd 		P_k1_k1(dim_state,dim_state);





	//_______next_iteration________

	Eigen::VectorXd			X_k_k(dim_state);
								
	Eigen::MatrixXd			P_k_k(dim_state,dim_state);

	Eigen::VectorXd			E_k_k(dim_measure);



void kalman_callback( const geometry_msgs::TransformStamped::ConstPtr& Transform_estimate)
{

		Y_k1[0] = Transform_estimate->transform.translation.x;
		Y_k1[1] = Transform_estimate->transform.translation.y;
		Y_k1[2] = Transform_estimate->transform.translation.z;
		Y_k1[3] = Transform_estimate->transform.rotation.w; 
		Y_k1[4] = Transform_estimate->transform.rotation.x;	
		Y_k1[5] = Transform_estimate->transform.rotation.y;			
		Y_k1[6] = Transform_estimate->transform.rotation.z;		



	//__________KALMAN_GAIN__________

		KALMAN = ( P_k1_k  * H_k.transpose() ) * ( ( H_k * P_k1_k * H_k.transpose() ) + R_k ).inverse() ;
	
	//___________UPDATE______________
		
		X_k1_k1 = ( I  - KALMAN * H_k) * X_k1_k  + ( KALMAN * Y_k1 ) ;
		P_k1_k1 = ( I  - KALMAN * H_k) * P_k1_k ;


}









int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "Kalman");// ROS node
   ros::NodeHandle nn;

	

	pose_KALMAN_pub = nn.advertise<geometry_msgs::PoseStamped>("/pose_KALMAN_estimation", 1);
	position_KALMAN_pub = nn.advertise<geometry_msgs::Vector3Stamped>("/position_KALMAN_estimation", 1);
	transform_KALMAN_pub = nn.advertise<geometry_msgs::TransformStamped>("/transform_KALMAN_estimation", 1);

	
	
	ros::Subscriber pose_obj_sub;
	pose_obj_sub = nn.subscribe("/transform_obj", 1, &kalman_callback);


	nn.param<std::string>("file", file_name, "prova");
	file_output_comparison.open("compare_" +  file_name +  ".txt",ofstream::app);
 
	P_filter.resize(dim_state);
	Q_process.resize(dim_state);
	R_sensor.resize(dim_measure);

   	nn.param<std::vector<double>>("P", P_filter, std::vector<double>{1,1,1,1,1,1,1,1,1,1,1,1,1,1});
	nn.param<std::vector<double>>("Q", Q_process, std::vector<double>{1,1,1,1,1,1,1,1,1,1,1,1,1,1});
	nn.param<std::vector<double>>("R", R_sensor, std::vector<double>{1,1,1,1,1,1,1});
    // per accedere al vettore usa [0]
   


	
	static tf::TransformBroadcaster br;





	//____________________________________INITIALIZATION FILTER____________________________________________
	//______________________________________________________________________________________________






	
	I <<  MatrixXd::Identity(dim_state,dim_state);	




	//transition_matrix ------ n x n

	F_k <<  1,   0   ,  0   ,  dt   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,
			  0,   1   ,  0   ,   0   ,  dt   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,
			  0,   0   ,  1   ,   0   ,   0   ,  dt   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,
			  0,   0   ,  0   ,   1   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,
			  0,   0   ,  0   ,   0   ,   1   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,
			  0,   0   ,  0   ,   0   ,   0   ,   1   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,
			  0,   0   ,  0   ,   0   ,   0   ,   0   ,   1   ,   0   ,   0   ,   0   ,   dt  ,   0   ,   0   ,   0   ,
			  0,   0   ,  0   ,   0   ,   0   ,   0   ,   0   ,   1   ,   0   ,   0   ,   0   ,   dt  ,   0   ,   0   ,
			  0,   0   ,  0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   1   ,   0   ,   0   ,   0   ,   dt  ,   0   ,
			  0,   0   ,  0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   1   ,   0   ,   0   ,   0   ,   dt  ,
			  0,   0   ,  0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   1   ,   0   ,   0   ,   0   ,
			  0,   0   ,  0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   1   ,   0   ,   0   ,
			  0,   0   ,  0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   1   ,   0   ,
			  0,   0   ,  0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   1   ;


	//output_matrix ---------- m_measure x n_state

	H_k <<   1  ,   0  ,  0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0	 ,   0   ,   0   ,   0   ,   0   ,
			 0  ,   1  ,  0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0  ,   0   ,   0   ,   0   ,   0   ,
			 0  ,   0  ,  1   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0  ,   0   ,   0   ,   0   ,   0   ,
			 0  ,   0  ,  0   ,   0   ,   0   ,   0   ,   1   ,   0   ,   0   ,   0  ,   0   ,   0   ,   0   ,   0   ,
			 0  ,   0  ,  0   ,   0   ,   0   ,   0   ,   0   ,   1   ,   0   ,   0  ,   0   ,   0   ,   0   ,   0   ,
			 0  ,   0  ,  0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   1   ,   0  ,   0   ,   0   ,   0   ,   0   ,
			 0  ,   0  ,  0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   0   ,   1  ,   0   ,   0   ,   0   ,   0   ;




	//covariance_matrix_state_error  --------   n_state x n_state


	P_k <<  P_filter[0],   0    ,  0     ,   0     ,   0     ,   0     ,   0     ,   0     ,   0     ,    0	 ,    0	   ,    0	  ,    0	    ,    0	 ,
				 0    , P_filter[1],  0     ,   0     ,   0     ,   0     ,   0     ,   0     ,   0     ,    0	 ,    0	   ,    0	  ,    0	    ,    0	 ,
				 0    ,   0    , P_filter[2],   0     ,   0     ,   0     ,   0     ,   0     ,   0     ,    0	 ,    0	   ,    0	  ,    0	    ,    0	 ,
				 0    ,   0    ,  0     , P_filter[3],  0     ,   0     ,   0     ,   0     ,   0     ,    0	 ,    0	   ,    0	  ,    0	    ,    0	 ,
				 0    ,   0    ,  0     ,   0     , P_filter[4],  0     ,   0     ,   0     ,   0     ,    0	 ,    0	   ,    0	  ,    0	    ,    0	 ,
				 0    ,   0    ,  0     ,   0     ,   0     , P_filter[5],  0     ,   0     ,   0     ,    0	 ,    0	   ,    0	  ,    0	    ,    0	 ,
				 0    ,   0    ,  0     ,   0     ,   0     ,   0     , P_filter[6],   0     ,   0     ,    0	 ,    0	   ,    0	  ,    0	    ,    0	 ,
				 0    ,   0    ,  0     ,   0     ,   0     ,   0     ,   0     , P_filter[7],   0     ,    0	 ,    0	   ,    0	  ,    0	    ,    0	 ,
				 0    ,   0    ,  0     ,   0     ,   0     ,   0     ,   0     ,   0     , P_filter[8],    0	 ,    0	   ,    0	  ,    0	    ,    0	 ,
				 0    ,   0    ,  0     ,   0     ,   0     ,   0     ,   0     ,   0     ,   0     , P_filter[9],    0	   ,    0	  ,    0	    ,    0	 ,
				 0    ,   0    ,  0     ,   0     ,   0     ,   0     ,   0     ,   0     ,   0     ,    0    ,P_filter[10],     0	  ,    0	    ,    0	 ,
				 0    ,   0    ,  0     ,   0     ,   0     ,   0     ,   0     ,   0     ,   0     ,    0    ,    0	   ,P_filter[11],    0	    ,    0	 ,
				 0    ,   0    ,  0     ,   0     ,   0     ,   0     ,   0     ,   0     ,   0     ,    0    ,    0	   ,    0	  ,P_filter[12],    0	 ,
				 0    ,   0    ,  0     ,   0     ,   0     ,   0     ,   0     ,   0     ,   0     ,    0    ,    0	   ,    0	  ,    0	    ,  P_filter[13]	 ;


	//covariance_matrix_model_error ------  n x n


	Q_k << Q_process[0],   0    ,   0     ,   0      ,   0      ,   0      ,   0	    ,    0    ,    0    ,    0	   ,    0	   ,    0	   ,    0	   ,    0	   ,
			    0   ,Q_process[1] ,   0     ,   0      ,   0      ,   0      ,   0		 ,    0    ,    0    ,    0	   ,    0	   ,    0	   ,    0	   ,    0	   ,
				 0   ,   0    , Q_process[2] ,   0      ,   0      ,   0      ,   0		 ,    0    ,    0    ,    0	   ,    0	   ,    0	   ,    0	   ,    0	   ,
				 0   ,   0    ,   0     , Q_process[3]  ,   0      ,   0      ,   0      ,    0    ,    0    ,    0	   ,    0	   ,    0	   ,    0	   ,    0	   ,
				 0   ,   0    ,   0     ,   0        , Q_process[4],   0      ,   0      ,    0    ,    0    ,    0	   ,    0	   ,    0	   ,    0	   ,    0	   ,
				 0   ,   0    ,   0     ,   0      ,   0      , Q_process[5]  ,   0      ,    0    ,    0    ,    0	   ,    0	   ,    0	   ,    0	   ,    0	   ,
				 0   ,   0    ,   0     ,   0      ,   0      ,   0      , Q_process[6]  ,    0    ,    0    ,    0		,    0	   ,    0	   ,    0	   ,    0	   ,
				 0   ,   0    ,   0     ,   0      ,   0      ,   0      ,   0     , Q_process[7]  ,    0    ,    0		,    0	   ,    0	   ,    0	   ,    0	   ,
				 0   ,   0    ,   0     ,   0      ,   0      ,   0      ,   0     ,    0    , Q_process[8]  ,    0		,    0	   ,    0	   ,    0	   ,    0	   ,
				 0   ,   0    ,   0     ,   0      ,   0      ,   0      ,   0     ,    0    ,   0     , Q_process[9]   ,     0	   ,    0	   ,    0	   ,    0	   ,
				 0   ,   0    ,   0     ,   0      ,   0      ,   0      ,   0     ,    0    ,   0     ,    0         ,Q_process[10] ,    0	   ,    0	   ,    0	   ,
				 0   ,   0    ,   0     ,   0      ,   0      ,   0      ,   0     ,    0    ,   0     ,    0		,    0	    ,Q_process[11]	,    0	   ,    0	   ,
				 0   ,   0    ,   0     ,   0      ,   0      ,   0      ,   0     ,    0    ,   0     ,    0		,    0	   ,    0	   ,Q_process[12]    ,    0	   ,
				 0   ,   0    ,   0     ,   0      ,   0      ,   0      ,   0     ,    0    ,   0     ,    0      ,    0	   ,    0	   ,    0	   ,Q_process[13];


	//covariance_matrix_sensor_noise ------ m x m

	R_k << R_sensor[0],   0      ,   0     ,   0     ,   0     ,   0     ,   0		,
				0    ,	R_sensor[1],   0     ,   0     ,   0     ,   0     ,   0		,
				0    ,    0     , R_sensor[2] ,   0     ,   0     ,   0     ,   0		,
			   0    ,    0     ,   0     , R_sensor[3] ,   0     ,   0     ,   0		,
				0    ,    0     ,   0     ,   0     , R_sensor[4] ,   0     ,   0		,
				0    ,    0     ,   0     ,   0     ,   0     , R_sensor[5] ,   0		, 
				0    ,    0     ,   0     ,   0     ,   0     ,   0     , R_sensor[6];
 


		P_k_k = P_k ; 



	X_k_k   << 0,0,0,0,0,0,1,0,0,0,0,0,0,0; // x y z dx dy dz qw qx qy qz // stato iniziale
	X_k1_k  << 0,0,0,0,0,0,1,0,0,0,0,0,0,0; // stato predetto
	X_k1_k1 << 0,0,0,0,0,0,1,0,0,0,0,0,0,0; // stato corretto	
	
	Y_k1 << 0,0,0,1,0,0,0; //iniz misura

		


		ros::Rate loop_rate(100);//RATE, non metterlo al massimo sennò occupo tutta la CPU ( no good 
		
		cout<<"dT = "<<dt<<endl;

	while(nn.ok())
	{	
		
	

	double trace_P_k_k_pos = 0;
	double trace_P_k_k_or = 0;	
	trace_P_k_k_pos = P_k_k(0,0) + P_k_k(1,1) + P_k_k(2,2);
	trace_P_k_k_or  = P_k_k(6,6) + P_k_k(7,7) + P_k_k(8,8) + P_k_k(9,9);


file_output_comparison<<X_k_k[0]<<' '<<X_k_k[1]<<' '<<X_k_k[2]<<' '<<X_k_k[6]<<' '<<X_k_k[7]<<' '<<X_k_k[8]<<' '<<X_k_k[9]<<' '<<Y_k1[0]<<' '<<Y_k1[1]<<' '<<Y_k1[2]<<' '<<Y_k1[3]<<' '<<Y_k1[4]<<' '<<Y_k1[5]<<' '<<Y_k1[6]<<' '<<trace_P_k_k_pos<<' '<<trace_P_k_k_or<<endl;


	//____________PREDICTION_________


		X_k1_k =   F_k * X_k_k ;
		P_k1_k = ( F_k * P_k_k * F_k.transpose() ) + Q_k ; //V ---> Q covar on process

	//________________________________


		
		ros::spinOnce();   // -------------------> update when i have a new measure 



		
		v_x = (X_k1_k1(0) - X_k_k(0)); 
		v_y = (X_k1_k1(1) - X_k_k(1));
		v_z = (X_k1_k1(2) - X_k_k(2));


		v_qw = (X_k1_k1(6) - X_k_k(6));
		v_qx = (X_k1_k1(7) - X_k_k(7));
		v_qy = (X_k1_k1(8) - X_k_k(8));
	   v_qz = (X_k1_k1(9) - X_k_k(9));




		X_k1_k1(3) = v_x;
		X_k1_k1(4) = v_y;
		X_k1_k1(5) = v_z;

		X_k1_k1(10) = v_qw;
		X_k1_k1(11) = v_qx;
		X_k1_k1(12) = v_qy;
		X_k1_k1(13) = v_qz;
			

		
		X_k_k = X_k1_k1;      
		P_k_k = P_k1_k1;

/*		


		// P_k_k next step

		if(no_measure)
		{	
			v_x = (X_k1_k(0) - X_k_k(0)); 
	   	v_y = (X_k1_k(1) - X_k_k(1));
		   v_z = (X_k1_k(2) - X_k_k(2));


		   v_qw = (X_k1_k(6) - X_k_k(6));
		   v_qx = (X_k1_k(7) - X_k_k(7));
		   v_qy = (X_k1_k(8) - X_k_k(8));
	      v_qz = (X_k1_k(9) - X_k_k(9));




		X_k1_k(3) = v_x;
		X_k1_k(4) = v_y;
		X_k1_k(5) = v_z;

		X_k1_k(10) = v_qw;
		X_k1_k(11) = v_qx;
		X_k1_k(12) = v_qy;
		X_k1_k(13) = v_qz;


			X_k_k = X_k1_k;			//passo successivo solo predizione
			P_k_k = P_k1_k; 
		}		
		else
		{
		v_x = (X_k1_k1(0) - X_k_k(0)); 
		v_y = (X_k1_k1(1) - X_k_k(1));
		v_z = (X_k1_k1(2) - X_k_k(2));


		v_qw = (X_k1_k1(6) - X_k_k(6));
		v_qx = (X_k1_k1(7) - X_k_k(7));
		v_qy = (X_k1_k1(8) - X_k_k(8));
	   v_qz = (X_k1_k1(9) - X_k_k(9));




		X_k1_k1(3) = v_x;
		X_k1_k1(4) = v_y;
		X_k1_k1(5) = v_z;

		X_k1_k1(10) = v_qw;
		X_k1_k1(11) = v_qx;
		X_k1_k1(12) = v_qy;
		X_k1_k1(13) = v_qz;
			

	

			X_k_k = X_k1_k1;       //correggo
			P_k_k = P_k1_k1; 
			no_measure = true;		
		}
*/
/*		X_k_k(6) = X_k_k(6) /  sqrt ( X_k_k(6)*X_k_k(6) + X_k_k(7)*X_k_k(7) + X_k_k(8)*X_k_k(8) + X_k_k(9)*X_k_k(9) ) ;
		X_k_k(7) = X_k_k(7) /  sqrt ( X_k_k(6)*X_k_k(6) + X_k_k(7)*X_k_k(7) + X_k_k(8)*X_k_k(8) + X_k_k(9)*X_k_k(9) ) ;
		X_k_k(8) = X_k_k(8) /  sqrt ( X_k_k(6)*X_k_k(6) + X_k_k(7)*X_k_k(7) + X_k_k(8)*X_k_k(8) + X_k_k(9)*X_k_k(9) ) ;
		X_k_k(9) = X_k_k(9) /  sqrt ( X_k_k(6)*X_k_k(6) + X_k_k(7)*X_k_k(7) + X_k_k(8)*X_k_k(8) + X_k_k(9)*X_k_k(9) ) ;
*/

/*

	
	std::cout<<"-------------PREVIOUS_ESTIMATED_STATE------------------"<< endl;
	std::cout<<  X_k_k << endl;
	std::cout<<"-------------------------------------------------------"<< endl;
	std::cout<<  P_k_k << endl;
	std::cout<<"-------------------------------------------------------"<< endl;




	

	std::cout<<"--------------------PREDICTION------------------"<< endl;
	std::cout<<  X_k1_k << endl;
	std::cout<<  P_k1_k << endl;
	std::cout<<"------------------------------------------------"<< endl;


	
	std::cout<<"---------------------MEASURE--------------------"<< endl;
	std::cout<<"Y_k1(0) : "<<  Y_k1(0) << endl;	//x
	std::cout<<"Y_k1(1) : "<<  Y_k1(1) << endl;	//y
	std::cout<<"Y_k1(2) : "<<  Y_k1(2) << endl;	//z
	std::cout<<"Y_k1(3) : "<<  Y_k1(3) << endl;	//qw
	std::cout<<"Y_k1(4) : "<<  Y_k1(4) << endl;	//qx
	std::cout<<"Y_k1(5) : "<<  Y_k1(5) << endl;	//qy
	std::cout<<"Y_k1(6) : "<<  Y_k1(6) << endl;	//qz
	std::cout<<"------------------------------------------------"<< endl;



	std::cout<<"--------------------KALMAN_GAIN ----------------"<< endl;
	std::cout<<  KALMAN << endl;
	std::cout<<"------------------------------------------------"<< endl;




	std::cout<<"------------------ESTIMATED_STATE----------------------"<< endl;
	std::cout<<  X_k1_k1 << endl;
	std::cout<<"-------------------------------------------------------"<< endl;
	std::cout<<  P_k1_k1 << endl;
	std::cout<<"-------------------------------------------------------"<< endl;



	E_k_k(0) = abs(X_k1_k1(0) - Y_k1[0]);
	E_k_k(1) = abs(X_k1_k1(1) - Y_k1[1]);
	E_k_k(2) = abs(X_k1_k1(2) - Y_k1[2]);
	E_k_k(3) = abs(X_k1_k1(6) - Y_k1[3]); 
	E_k_k(4) = abs(X_k1_k1(7) - Y_k1[4]); 
	E_k_k(5) = abs(X_k1_k1(8) - Y_k1[5]); 
	E_k_k(6) = abs(X_k1_k1(9) - Y_k1[6]); 


	std::cout<<"-------------ERROR_STEP___KALMAN-MEASURE------------------"<< endl;
	std::cout<<" E_k_k(0) = " <<  E_k_k(0)  << endl; 
	std::cout<<" E_k_k(1) = " <<  E_k_k(1)  << endl; 
	std::cout<<" E_k_k(2) = " <<  E_k_k(2)  << endl; 
	std::cout<<" E_k_k(3) = " <<  E_k_k(3)  << endl;
	std::cout<<" E_k_k(4) = " <<  E_k_k(4)  << endl;
	std::cout<<" E_k_k(5) = " <<  E_k_k(5)  << endl;
	std::cout<<" E_k_k(6) = " <<  E_k_k(6)  << endl;
	std::cout<<"----------------------------------------------------------"<< endl;


*/


	//____ESTIMATED STATE_____________________________________________________________________________

	tf::Vector3 traslazione_EST(X_k_k[0], X_k_k[1], X_k_k[2]);  						   //  x  y  z
	tf::Quaternion rotazione_EST(X_k_k[7], X_k_k[8], X_k_k[9], X_k_k[6]);				// qx qy qz qw
	tf::Transform trasformazione_KALMAN_estimation(rotazione_EST, traslazione_EST);
	tf::StampedTransform stampedTransform_KALMAN_estimation(trasformazione_KALMAN_estimation, ros::Time::now(), "G_optical_frame", "/KALMAN_ESTIMATION_" +  file_name);


	
	br.sendTransform(stampedTransform_KALMAN_estimation);	//BROADCAST TF 


	




//____SEND_ON_TOPIC__________________________________________________________________________________
//___________________________________________________________________________________________________

			//PUBLISH /pose      
			geometry_msgs::PoseStamped poseMsg;
			tf::poseTFToMsg(trasformazione_KALMAN_estimation, poseMsg.pose);
			poseMsg.header.frame_id = "/object_frame_KALMAN_ESTIMATION" +  file_name;
			poseMsg.header.stamp    = ros::Time::now();
			pose_KALMAN_pub.publish(poseMsg);


			//PUBLISH /transform
			geometry_msgs::TransformStamped transformMsg;
			tf::transformStampedTFToMsg(stampedTransform_KALMAN_estimation, transformMsg);
			transform_KALMAN_pub.publish(transformMsg);


			//PUBLISH position
			geometry_msgs::Vector3Stamped positionMsg;
			positionMsg.header = transformMsg.header;
			positionMsg.vector = transformMsg.transform.translation;
			position_KALMAN_pub.publish(positionMsg);


//____________________________________________________________________________________________________
//____________________________________________________________________________________________________


		loop_rate.sleep();

	} // ___ close while


	file_output_comparison.close();
	

} // ___ close main
  




