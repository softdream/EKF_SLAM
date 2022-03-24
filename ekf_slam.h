#ifndef __EKF_SLAM_H
#define __EKF_SLAM_H

#include "motion.h"
#include "observation.h"
#include "utils.h"

namespace slam
{

class EKFSlam
{
public:
	EKFSlam();
	
	EKFSlam( const cv::Mat &K, const cv::Mat &dist, const int n_markers, const int marker_size, const double marker_length, const Eigen::Matrix4d T_r_c );
	
	~EKFSlam();	


	void predictByEncoder( const double &enl, const double &enr );
	void predictByOdometry(  );

	void updateByImageAruco( const cv::Mat &image );
	
private:
	Observation *observe;

	bool is_init_ = false;
	
	double k_ = 0.01; // 里程计协方差参数
    	double k_r_ = 0.05; // 观测协方差参数
    	double k_phi_ = 0.05; // 观测协方差参数

	/* 上一帧的编码器读数 */
    	double last_enl_, last_enr_;

	Eigen::MatrixXd x_; // state
	Eigen::MatrixXd sigma_ ; // covarince
	
	std::vector<int> aruco_ids_; //对应于每个路标的aruco码id 
};

}

#endif
