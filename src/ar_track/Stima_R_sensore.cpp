#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <unsupported/Eigen/MatrixFunctions>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <cmath>



using namespace std;
using namespace Eigen;


int samples    = 100;
int sampling   = 0;
int dim_misure = 7;

//__________________________________ x y z qw qx qy qz

Eigen::VectorXd 		media(dim_misure);  		

Eigen::VectorXd 		varianza(dim_misure);		

Eigen::VectorXd 		app(dim_misure);

Eigen::MatrixXd 		Y_k(samples,dim_misure);



void stima_callback( const geometry_msgs::TransformStamped::ConstPtr& Transform_estimate)
{

		if( sampling < samples )
		{

		cout << " dentro call" << endl;
	
		Y_k(sampling,0) = Transform_estimate->transform.translation.x;
		Y_k(sampling,1) = Transform_estimate->transform.translation.y;
		Y_k(sampling,2) = Transform_estimate->transform.translation.z;
		Y_k(sampling,3) = Transform_estimate->transform.rotation.w; 
		Y_k(sampling,4) = Transform_estimate->transform.rotation.x;	
		Y_k(sampling,5) = Transform_estimate->transform.rotation.y;			
		Y_k(sampling,6) = Transform_estimate->transform.rotation.z;		


		cout << " dopo " << endl;


		
			sampling++;
		
			cout << " sampling_step " << sampling ;
		}
		else
		{
			cout<<" calcolo media e varianza " << endl;


			for(int i = 0 ; i < samples ; i++)
			{
					media[0] = media[0] + ( Y_k(i,0) - media[0] ) / (i+1);
					media[1] = media[1] + ( Y_k(i,1) - media[1] ) / (i+1);
					media[2] = media[2] + ( Y_k(i,2) - media[2] ) / (i+1);
					media[3] = media[3] + ( Y_k(i,3) - media[3] ) / (i+1);
					media[4] = media[4] + ( Y_k(i,4) - media[4] ) / (i+1);
					media[5] = media[5] + ( Y_k(i,5) - media[5] ) / (i+1);
					media[6] = media[6] + ( Y_k(i,6) - media[6] ) / (i+1);
			}

		

			for(int i = 0 ; i < samples ; i++)
			{
				app[0]  =  abs ( Y_k(i,0) - media[0] );		
				app[1]  =  abs ( Y_k(i,1) - media[1] );
				app[2]  =  abs ( Y_k(i,2) - media[2] );
				app[3]  =  abs ( Y_k(i,3) - media[3] );
				app[4]  =  abs ( Y_k(i,4) - media[4] );
				app[5]  =  abs ( Y_k(i,5) - media[5] );
				app[6]  =  abs ( Y_k(i,6) - media[6] );


				if( app[0] > varianza[0] )  varianza[0]  =  app[0];	
				if( app[1] > varianza[1] )	varianza[1]  =  app[1];
				if( app[2] > varianza[2] )	varianza[2]  =  app[2];
				if( app[3] > varianza[3] )	varianza[3]  =  app[3];
				if( app[4] > varianza[4] )	varianza[4]  =  app[4];
				if( app[5] > varianza[5] )	varianza[5]  =  app[5];
				if( app[6] > varianza[6] )	varianza[6]  =  app[6];

			}

			cout << "---------------MEDIA--------------------" << endl;
			cout << media << endl;
			cout << "----------------------------------------" << endl;

			cout << "---------------VARIANZA-----------------" << endl;
			cout << varianza << endl;
			cout << "________________________________________" << endl;


		}
}



int main(int argc, char** argv)
{

	ros::init(argc, argv, "Stima");// ROS node
   ros::NodeHandle nn;

	
	cout << " qui " << endl;

	ros::Subscriber pose_obj_sub;
	pose_obj_sub = nn.subscribe("/transform_obj", 1, &stima_callback);


	cout << " qui 2 " << endl;


	Y_k = MatrixXd::Zero(samples, dim_misure);


	cout << " qui 3 " << endl;

	app  		<< 0,0,0,0,0,0,0;
	media		<< 0,0,0,0,0,0,0;
	varianza    << 0,0,0,0,0,0,0;


	cout << " qui 4 " << endl;	

	ros::Rate loop_rate(100);


	cout << " qui 5 " << endl;


	while(nn.ok())
	{


	//cout << " qui 6 " << endl;
			
		ros::spinOnce(); 
		loop_rate.sleep();

	//cout << " qui 7 " << endl;

	}

}
