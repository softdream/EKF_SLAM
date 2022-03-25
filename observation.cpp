#include "observation.h"

#include <algorithm>

namespace slam
{

Observation::Observation()
{

}

Observation::Observation( const cv::Mat &K, const cv::Mat &dist, const int n_markers, const int marker_size, const double marker_length )
                        : K_( K ),
                          dist_( dist ),
                          n_markers_( n_markers ),
                          marker_size_( marker_size ),
                          marker_length_( marker_length )
{
        dictionary_ = cv::aruco::generateCustomDictionary(n_markers_, marker_size_);
}


Observation::Observation( const cv::Mat &K, const cv::Mat &dist, const int n_markers, const int marker_size, const double marker_length, const Eigen::Matrix4d T_r_c ) 
			: K_( K ),
			  dist_( dist ),
			  n_markers_( n_markers ),
		  	  marker_size_( marker_size ),
			  marker_length_( marker_length ),
			  T_r_c_( T_r_c )
{
	dictionary_ = cv::aruco::generateCustomDictionary(n_markers_, marker_size_);
}

Observation::~Observation()
{

}

const int Observation::getMarkers( const cv::Mat &image, std::vector<Marker> &markers )
{
	markers.clear();

	std::vector<std::vector<cv::Point2f>> marker_corners;
	std::vector<int> ArucoIDs;
	std::vector<cv::Vec3d> rvs, tvs;
	cv::aruco::detectMarkers(image, dictionary_ , marker_corners, ArucoIDs);
	cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_length_, K_, dist_, rvs, tvs);

	/* draw all marks */
    	marker_img_ = image.clone();
    	cv::aruco::drawDetectedMarkers(marker_img_, marker_corners, ArucoIDs);
    	
	for(size_t i = 0; i < ArucoIDs.size(); i ++){
        	cv::aruco::drawAxis(marker_img_, K_, dist_, rvs[i], tvs[i], 0.14);
	}

	for( size_t i = 0; i < ArucoIDs.size(); i ++ ){
		double dist = cv::norm<double>(tvs[i]); //计算距离
		if( dist > DIST_THRESHOLD ) {
			continue;
		}
		
		Eigen::Vector2d rangeBearing = convert2RangeBearing( tvs[i], rvs[i] );
		markers.push_back( Marker( ArucoIDs[i], rangeBearing[0], rangeBearing[1] ) );
	}

	return markers.size();
}

const Eigen::Vector2d Observation::convert2RangeBearing( const cv::Vec3d &tvec, const cv::Vec3d &rvec ) const
{
	cv::Mat R;
	cv::Rodrigues(rvec, R);
        Eigen::Matrix4d T_c_m;
        
	T_c_m <<
        R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), tvec[0],
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), tvec[1],
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), tvec[2],
        0.,0.,0.,1.;

	Eigen::Matrix4d T_r_m = T_r_c_ * T_c_m;
	
	double& x = T_r_m(0, 3);
        double& y = T_r_m(1, 3);

        double r = ::sqrt(x*x + y*y);
        double phi = ::atan2(y, x);
	
	return Eigen::Vector2d( r, phi );	
}

void Observation::showMarkersInCurrentObservation( const std::vector<Marker> &markers ) const
{
	cv::Mat image = cv::Mat::zeros(CANVAS_WIDTH, CANVAS_HEIGHT, CV_8UC3);
	
	cv::circle( image, cv::Point( CANVAS_CENTER_X, CANVAS_CENTER_Y ), 3, cv::Scalar(0, 0, 255), -1 );

	cv::arrowedLine( image, cv::Point2f( CANVAS_CENTER_X, CANVAS_CENTER_Y ), cv::Point2f( CANVAS_CENTER_X, CANVAS_CENTER_Y - 40 ), cv::Scalar( 0, 0, 255 ), 2 );
        cv::arrowedLine( image, cv::Point2f( CANVAS_CENTER_X, CANVAS_CENTER_Y ), cv::Point2f( CANVAS_CENTER_X - 40, CANVAS_CENTER_Y ), cv::Scalar( 0, 0, 255 ), 2 );


	for( size_t i = 0; i < markers.size(); i ++ ){
		double x = markers[i].r_ * ::cos( markers[i].phi_ );
		double y = markers[i].r_ * ::sin( markers[i].phi_ );

		double img_x = CANVAS_CENTER_X - y * MARKER_X_SCALAR;
		double img_y = CANVAS_CENTER_Y - x * MARKER_Y_SCALAR;
	
		cv::putText(image, "id: " + std::to_string( markers[i].aruco_id_ ), cv::Point(img_x, img_y - 20), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255, 0, 0), 1);
		
		cv::circle( image, cv::Point2d( img_x, img_y ), 6, cv::Scalar( 0, 255, 0 ), -1 );
	
		cv::line( image, cv::Point2d( CANVAS_CENTER_X, CANVAS_CENTER_Y ), cv::Point2d( img_x, img_y ), cv::Scalar( 0, 255, 0 ), 1 );
		
		cv::putText(image, "(" + std::to_string( markers[i].r_ ) + "," + std::to_string( markers[i].phi_ ) + ")", cv::Point2d( (img_x + CANVAS_CENTER_X) / 2 , (img_y + CANVAS_CENTER_Y) / 2 ), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 0, 255), 1);

	}

	cv::imshow("Current Observation", image);
}

bool Observation::findLandMarker( std::vector<int> &ids, const int aruco_id, int &landmarker_index )
{
	// binary search
	std::sort( ids.begin(), ids.end() );

	int left = 0;
	int right = ids.size();
	
	while( left < right ){
		int mid = left + ( right - left ) * 0.5;
		
		if( aruco_id > ids[mid] ){
			left = mid + 1;
		}
		else if( aruco_id < ids[mid] ){
			right = mid - 1;
		}
		else {
			landmarker_index = mid;
			return true;
		}
	}	
	
	landmarker_index = -1;
	return false;
}

}

