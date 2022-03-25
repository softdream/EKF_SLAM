#include "observation.h"
#include <iostream>
#include "utils.h"
#include "motion.h"
#include "ekf_slam.h"

#include <thread>

slam::EKFSlam *ekf;
bool start_flag = false;
bool image_get = false;

void motion_thread_function()
{
	std::cout<<"----------- Motion ------------"<<std::endl;
	slam::Motion::openASimulationFile( "encoder_data.txt" );

        std::vector<slam::Motion> motionsVec;

	while( slam::Motion::getFrameCount() < 2000 ){
		double enl = 0;
                double enr = 0;
                int frame = slam::Motion::readAFrameData( enl, enr );
		std::cout<<"encoder frame : "<<frame<<std::endl;
		//slam::Motion motion_data = slam::Motion::getMotionValueFromEncoder( enl, enr, last_enl, last_enr );
		if( frame == 2 ){
			std::cout<<"start image read ..."<<std::endl;
			start_flag = true;
		}

		ekf->predictByEncoder( enl, enr );	
		cv::Mat image = cv::Mat::zeros(600, 600, CV_8UC3);
		ekf->showMarkers( image );
		cv::imshow("map", image);

		if( image_get )
			ekf->showMarkedImg();
		cv::waitKey(39);
		//std::this_thread::sleep_for(std::chrono::milliseconds(40));//单位是毫秒
	}

	slam::Motion::closeSimulationFile();
}

void observe_thread_function()
{
	std::cout<<"----------- Observe ------------"<<std::endl;
	int img_count = 0;
	
	while( !start_flag );

	while( img_count < 800 ){
                std::string path = "/home/riki/dataset/" + std::to_string( img_count ) + ".jpg";
                cv::Mat image = cv::imread( path );
                if( image.empty() ) break;
        	image_get = true; 
	        std::cout<<"image serial : "<<img_count<<std::endl;

                //cv::imshow( "source image", image );

		ekf->updateByImageAruco( image );
		
                img_count ++;
                std::this_thread::sleep_for(std::chrono::milliseconds(93));//单位是毫秒
	}

}

int main()
{
	std::cout<<"---------------- Test --------------"<<std::endl;

	std::cout<<"=============== GET PARAMETERS =============="<<std::endl;
	slam::Utils::openYamlFile( "./config.yaml" );
	double fx;
	slam::Utils::readParameter( "camera", "fx", fx );

	double fy;
	slam::Utils::readParameter( "camera", "fy", fy );
	
	double cx;
        slam::Utils::readParameter( "camera", "cx", cx );

	double cy;
        slam::Utils::readParameter( "camera", "cy", cy );

	double k1;
        slam::Utils::readParameter( "camera", "k1", k1 );

	double k2;
        slam::Utils::readParameter( "camera", "k2", k2 );
	
	double p1;
        slam::Utils::readParameter( "camera", "p1", p1 );
	
	double p2;
        slam::Utils::readParameter( "camera", "p2", p2 );

	double k3;
        slam::Utils::readParameter( "camera", "k3", k3 );

	int n_markers;
	slam::Utils::readParameter( "aruco", "n_markers", n_markers );

	int marker_size;
        slam::Utils::readParameter( "aruco", "marker_size", marker_size );

	double marker_length;
        slam::Utils::readParameter( "aruco", "marker_length", marker_length );
	
	Eigen::Matrix4d T_r_c;
	T_r_c << 0.0, 0.258819, 0.965926, 0.18, 
		 -1.0, 0.0, 0.0, -0.1, 
		 0.0, -0.965926, 0.258819, 0.0, 
		 0.0, 0.0, 0.0, 1.0;
	//slam::Utils::readParameterEigen( "Trc", T_r_c );

	slam::Utils::closeYamlFile();
	
	std::cout<<"camera : "<<std::endl;
	std::cout<<"fx            = "<<fx<<std::endl;
	std::cout<<"fy            = "<<fy<<std::endl;
	std::cout<<"cx            = "<<cx<<std::endl;
	std::cout<<"cy            = "<<cy<<std::endl;
	std::cout<<"k1            = "<<k1<<std::endl;
	std::cout<<"k2            = "<<k2<<std::endl;
	std::cout<<"p1            = "<<p1<<std::endl;
	std::cout<<"p2            = "<<p2<<std::endl;
	std::cout<<"k3            = "<<k3<<std::endl;
	std::cout<<"aruco : "<<std::endl;
	std::cout<<"n_markers     = "<<n_markers<<std::endl;
	std::cout<<"marker_size   = "<<marker_size<<std::endl;
	std::cout<<"marker_length = "<<marker_length<<std::endl;
	std::cout<<"T_r_c         = "<<std::endl<<T_r_c<<std::endl;
	
	cv::Mat K = (cv::Mat_<float>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    	cv::Mat dist = (cv::Mat_<float>(5, 1) << k1, k2, p1, p2, k3);
	
	std::cout<<"=============== INIT EKF_LAM =============="<<std::endl;
	ekf = new slam::EKFSlam( K, dist, n_markers, marker_size, marker_length, T_r_c );

	std::cout<<"=============== START SLAM =============="<<std::endl;
	std::thread motion_thread( motion_thread_function );		
	std::thread observe_thread( observe_thread_function );

	motion_thread.join();
	observe_thread.join();

	return 0;
}
