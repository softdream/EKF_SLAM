#include "motion.h"

#include <iostream>

namespace slam
{

std::ifstream Motion::simu_file;

int Motion::frame_count = 0;

const Motion Motion::getMotionValueFromEncoder( const double enl, const double enr, const double last_enl, const double last_enr )
{
	/***** 编码器数据预处理 *****/
    	/* 计算 Delta_l/r */
    	double delta_enl = enl - last_enl;
    	double delta_enr = enr - last_enr;
	//std::cout<<"enl = "<<enl<<", enr = "<<enr<<std::endl;
	//std::cout<<"last enl = "<<last_enl<<", last_enr = "<<last_enr<<std::endl;	

	//std::cout<<"delta enl = "<<delta_enl<<", delta enr = "<<delta_enr<<std::endl;

    	double delta_sl = 2.31249273072714e-06 * delta_enl;
    	double delta_sr = 2.31249273072714e-06 * delta_enr;

    	/* 计算 Delta theta and Delta s */
    	double delta_theta = (delta_sr - delta_sl) / b_;
	//std::cout<<"delta_sr = "<<delta_sr<<", delta_sl = "<<delta_sl<<std::endl;
	//std::cout<<"delta_sr - delta_sl = "<<delta_sr - delta_sl<<std::endl;
//	std::cout<<"delta_theta = "<<delta_theta<<", b = "<<b_<<std::endl;
	//std::cout<<"--------------------------------"<<std::endl;
    	double delta_s = 0.5 * (delta_sr + delta_sl);

	return Motion( delta_s, delta_theta );
}

bool Motion::openASimulationFile( const std::string &file_name )
{
	simu_file.open( file_name.c_str(), std::ifstream::in );

	if( !simu_file.is_open() ){
		std::cerr<<"Failed to open the simulation file !"<<std::endl;
		return false;
	}

	return true;
}

const int Motion::readAFrameData( double &enl, double &enr )
{	
	std::string line;
	
	std::getline( simu_file, line );
	{
		std::istringstream iss( line );

		std::string tag;
		iss >> tag;

		std::string num;	
		
		if( tag.compare( "encoder" ) == 0 ){
			iss >> num;
			enl = std::stod( num );
			
			iss >> num;
			enr = std::stod( num );

			frame_count ++;
		}
	}

	

	return frame_count;
}

void Motion::closeSimulationFile()
{	
	return simu_file.close();
}

void Motion::showTrajectory( const std::vector<Motion> &motions )
{
	cv::Mat image = cv::Mat::zeros(600, 600, CV_8UC3);
	
	cv::circle( image, cv::Point( 300, 300 ), 3, cv::Scalar(0, 0, 255), -1 );
	Eigen::Vector3d pose_old( 0, 0, 0 );
	Eigen::Vector3d pose_now;

	for( size_t i = 0; i < motions.size(); i ++ ){
		Motion data = motions[i];
		std::cout<<"delta s = "<<data.delta_s_<<", delta theta = "<<data.delta_theta_<<std::endl;
		
		double cos_tmp = ::cos( pose_old[2] + 0.5 * data.delta_theta_ );
		double sin_tmp = ::sin( pose_old[2] + 0.5 * data.delta_theta_ );
		pose_now[0] = pose_old[0] + data.delta_s_ * cos_tmp;
		pose_now[1] = pose_old[1] + data.delta_s_ * sin_tmp;

		pose_now[2] = pose_old[2] + data.delta_theta_;

		// draw 
		double img_x_old = 300 - pose_old[1] * 50 ;
		double img_y_old = 300 - pose_old[0] * 50;
		double img_x_now = 300 - pose_now[1] * 50;
                double img_y_now = 300 - pose_now[0] * 50;
		std::cout<<"pose_now.x = "<<pose_now[0]<<", pose_now.y = "<<pose_now[1]<<std::endl;
		cv::line( image, cv::Point2d( img_x_old, img_y_old ), cv::Point2d( img_x_now, img_y_now ), cv::Scalar( 0, 0, 255 ), 1 );

		cv::imshow( "Trajectory", image );
		cv::waitKey( 30 );

		pose_old = pose_now;
	}
	
	cv::imwrite("trajectory.jpg", image);
}

}
