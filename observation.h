#ifndef __OBSERVATION_H
#define __OBSERVATION_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#define DIST_THRESHOLD 3.0
#define MARKER_X_SCALAR 120
#define MARKER_Y_SCALAR 120

#define CANVAS_WIDTH 600
#define CANVAS_HEIGHT 600

#define CANVAS_CENTER_X 300
#define CANVAS_CENTER_Y 400

namespace slam
{

struct Marker
{
        Marker()
        {

        }

        Marker( const int aruco_id, const double r, const double phi )
                   : aruco_id_( aruco_id ),
                     r_( r ),
                     phi_( phi )
        {

        }

        int aruco_id_;
        double r_;
        double phi_;
};


class Observation
{
public:
	Observation();
	Observation( const cv::Mat &K,
                     const cv::Mat &dist,
                     const int n_markers,
                     const int marker_size,
                     const double marker_length);

	Observation( const cv::Mat &K, 
		     const cv::Mat &dist, 
		     const int n_markers, 
		     const int marker_size, 
		     const double marker_length,
	 	     const Eigen::Matrix4d T_r_c );
	
	~Observation();
	
	const int getMarkers( const cv::Mat &image, std::vector<Marker> &markers );
	void showMarkersInCurrentObservation( const std::vector<Marker> &markers ) const;	

	inline const cv::Mat& getMarkeredImg() const
	{	
		return marker_img_;
	}

	static bool findLandMarker( std::vector<int> &ids, const int aruco_id, int &landmarker_index );

private:
	const Eigen::Vector2d convert2RangeBearing( const cv::Vec3d &tvec, const cv::Vec3d &rvec ) const;

private:
	/* 系统配置参数 */
    	cv::Mat K_, dist_; //　相机内参数

	cv::Ptr<cv::aruco::Dictionary> dictionary_;
	
	cv::Mat marker_img_;

	int n_markers_;
    	int marker_size_;

	double marker_length_;

	Eigen::Matrix4d T_r_c_; // 机器人外参数
};

}

#endif
