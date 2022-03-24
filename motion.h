#ifndef __MOTION_H
#define __MOTION_H


#include <fstream>
#include <string.h>
#include <sstream>
#include <iomanip>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <vector>

namespace slam
{

struct Motion
{
	Motion( const double delta_s, const double delta_theta ) : 
		delta_s_( delta_s ), 
		delta_theta_( delta_theta )
	{

	}

	~Motion()
	{

	}

	static void showTrajectory( const std::vector<Motion> &motions );

	static const Motion getMotionValueFromEncoder( const double enl, const double enr, const double last_enl, const double last_enr );
	static const Motion getMotionValueFromOdometry();

	static bool openASimulationFile( const std::string &file_name );
	static const int readAFrameData( double &enl, double &enr );
	static const int readAFrameData(  );
	static void closeSimulationFile();

	inline static const int filePointPose()
        {
                return simu_file.tellg();
        }

        inline static const int endOfFile()
        {
                return simu_file.eof();
        }

        inline static const int getFrameCount()
        {
                return frame_count;
        }


	double delta_s_ = 0;
	double delta_theta_ = 0;
	
	static constexpr double b_ = 0.719561485930414;
	static std::ifstream simu_file;
	static int frame_count;
};

}

#endif
