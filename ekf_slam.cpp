#include "ekf_slam.h"


namespace slam
{

EKFSlam::EKFSlam()
{
	x_.resize( 3, 1 );
	x_.setZero();
	
	sigma_.resize( 3, 3 );
	sigma_.setZero();

	observe = nullptr;

}

EKFSlam::EKFSlam( const cv::Mat &K, const cv::Mat &dist, const int n_markers, const int marker_size, const double marker_length, const Eigen::Matrix4d T_r_c )
{
	x_.resize( 3, 1 );
        x_.setZero();

        sigma_.resize( 3, 3 );
        sigma_.setZero();

	observe = new Observation( K, dist, n_markers, marker_size, marker_length, T_r_c );
}

EKFSlam::~EKFSlam()
{
	
}

void EKFSlam::predictByEncoder( const double &enl, const double &enr )
{
	if( is_init_ == false ){
		last_enl_ = enl;
		last_enr_ = enr;
		
		is_init_ = true;
		return;
	} 

	Motion data = Motion::getMotionValueFromEncoder( enl, enr, last_enl_, last_enr_ );

	// state predict
	double tmp_theta = x_(2, 0) + 0.5 * data.delta_theta_;
	double cos_tmp = ::cos( tmp_theta );
	double sin_tmp = ::sin( tmp_theta );

	x_(0, 0) += data.delta_s_ * cos_tmp;
	x_(1, 0) += data.delta_s_ * sin_tmp;
	x_(2, 0) += data.delta_theta_;

	Utils::normalizeAngle( x_(2, 0) );

	// State covariance predict
	Eigen::Matrix3d G_cauchy;
	G_cauchy << 1.0, 0.0, -data.delta_s_ * sin_tmp,
		    0.0, 1.0, data.delta_s_ * cos_tmp,
		    0.0, 0.0, 1.0;

	// --------------------------------------------------//
	// control covariance predict
	Eigen::Matrix<double, 3, 2> G_u_p;
	G_u_p << cos_tmp, -0.5 * data.delta_s_ * sin_tmp,
		 sin_tmp, 0.5 * data.delta_s_ * cos_tmp,
		 0      , 1;
	
	/*G_u_p << 0.5  * (cos_tmp - data.delta_s_ * sin_tmp / 0.719561485930414), 0.5  * (cos_tmp + data.delta_s_ * sin_tmp / 0.719561485930414),
   	       0.5  * (sin_tmp + data.delta_s_ * cos_tmp / 0.719561485930414), 0.5  *(sin_tmp - data.delta_s_ * cos_tmp / 0.719561485930414),
   	       1.0 / 0.719561485930414, -1.0 / 0.719561485930414;
	*/
	// -------------------------------------------------//
	
	int N = x_.rows();
	std::cout<<"state vector size = "<<N<<std::endl;
	Eigen::MatrixXd F( N, 3 );
	F.setZero();
	F.block( 0, 0, 3, 3 ) = Eigen::Matrix3d::Identity();
	//std::cout<<"F matrix = "<<std::endl<<F<<std::endl;
	

	// consider the landmarkers
	Eigen::MatrixXd Gt = Eigen::MatrixXd::Identity(N, N);
	Gt.block(0, 0, 3, 3) = G_cauchy;
	//std::cout<<"Gt matrix = "<<std::endl<<Gt<<std::endl;

	// odometry covarince
	Eigen::Matrix2d sigma_u;
	sigma_u << k_ * k_ * data.delta_s_, 0.0,
		   0.0, k_ * k_ * data.delta_theta_;
	//std::cout<<"sigma_u = "<<std::endl<<sigma_u<<std::endl;	

	// covariance matrix predict
	sigma_ = Gt * sigma_ * Gt.transpose() + F * G_u_p * sigma_u * G_u_p.transpose() * F.transpose();

	//std::cout<<"x_ = "<<std::endl<<x_<<std::endl;
	//std::cout<<"sigma_= "<<std::endl<<sigma_<<std::endl;
	std::cout<<"------------------------------------"<<std::endl;

	// 
	last_enl_ = enl;
	last_enr_ = enr;
}

void EKFSlam::updateByImageAruco( const cv::Mat &image )
{
	if( is_init_ == false ){
		return;
	}

	std::vector<Marker> markers;
	observe->getMarkers( image, markers );
	
	for( auto it : markers ){
		// observe covarince
		Eigen::Matrix2d Q;
		Q << k_r_ * k_r_ * std::fabs( it.r_ * it.r_ ), 0.0,
		     0.0, k_phi_ * k_phi_ * std::fabs( it.phi_ * it.phi_ );
		std::cout<<"aruco_id = "<<it.aruco_id_<<std::endl;
		int i; // the i th landmarker
		//if( Observation::findLandMarker( aruco_ids_, it.aruco_id_, i ) ){
		if( checkLandmark(it.aruco_id_, i) ) {
			// if it is a landmarker that already exists
			int N = x_.rows();
			Eigen::MatrixXd F( 5, N );
			F.setZero();	
			
			F.block( 0, 0, 3, 3 ) = Eigen::Matrix3d::Identity();
			F(3, 3 + 2 * i) = 1;
			F(4, 4 + 2 * i) = 1;
			
			double m_x = x_( 3 + 2 * i, 0 );
			double m_y = x_( 4 + 2 * i, 0 );	
			double x = x_( 0, 0 );
			double y = x_( 1, 0 );
			double theta = x_( 2, 0 );
			
			double delta_x = m_x - x;
			double delta_y = m_y - y;
			
			double q = delta_x * delta_x + delta_y * delta_y;
			double sqrt_q = ::sqrt( q );
		
			Eigen::MatrixXd H_v(2, 5);
			H_v << -sqrt_q * delta_x, -sqrt_q * delta_y, 0, sqrt_q * delta_x, sqrt_q * delta_y,
			       delta_y, -delta_x, -q, -delta_y, delta_x;
			
			H_v = ( 1 / q ) * H_v;

			Eigen::MatrixXd H_t = H_v * F;
	
			// Kalman Filter
			Eigen::MatrixXd K = sigma_ * H_t.transpose() * ( H_t * sigma_ * H_t.transpose() + Q ).inverse(); // Kalman Gain
			
			double phi_hat = ::atan2( delta_y, delta_x ) - theta;
			Utils::normalizeAngle( phi_hat );
			
			Eigen::Vector2d z_hat( sqrt_q, phi_hat );

			Eigen::Vector2d z( it.r_, it.phi_ );
			
			x_ = x_ + K * ( z - z_hat );
			Eigen::MatrixXd I = Eigen::MatrixXd::Identity( N, N );
			sigma_ = ( I - K * H_t ) * sigma_;
		}	
		else {
			std::cout<<"add new landmarkers ..............."<<std::endl;
			// add new landmarkers
			double angle = x_(2, 0) + it.phi_;
			Utils::normalizeAngle( angle );

			double m_x = it.r_ * ::cos( angle ) + x_(0, 0);
			double m_y = it.r_ * ::sin( angle ) + x_(1, 0);

			Eigen::Matrix3d sigma_cauchy = sigma_.block( 0, 0, 3, 3 );
						
			Eigen::Matrix<double, 2, 3> Gp;
			Gp << 1.0, 0.0, -it.r_ * ::sin(angle),
			      0.0, 1.0, it.r_ * ::cos(angle);

			Eigen::Matrix2d Gz;
			Gz << ::cos(angle), -it.r_ * ::sin(angle),
			      ::sin(angle), it.r_ * ::cos(angle);

			Eigen::Matrix2d sigma_m = Gp * sigma_cauchy * Gp.transpose() + Gz * Q * Gz.transpose();
			
			Eigen::MatrixXd Gfx;
			Gfx.resize( 2, x_.rows() );
			Gfx.setZero();
			Gfx.block( 0, 0, 2, 3 ) = Gp;

			Eigen::MatrixXd sigma_mx;
			sigma_mx.resize( 2, x_.rows() );
			sigma_mx.setZero();
			sigma_mx = Gfx * sigma_;

			// expand the state
			int N = x_.rows();
			Eigen::MatrixXd tmp_x( N + 2, 1 ); // add a new landmarker
			tmp_x.setZero();
			tmp_x << x_, m_x, m_y;
			x_.resize( N + 2, 1 );
			x_ = tmp_x;

			// expand the state covarince
			Eigen::MatrixXd tmp_sigma( N + 2, N + 2 );
			tmp_sigma.setZero();
			tmp_sigma.block( 0, 0, N, N ) = sigma_;
			tmp_sigma.block( N, N, 2, 2 ) = sigma_m;
			tmp_sigma.block( N, 0, 2, N ) = sigma_mx;
			tmp_sigma.block( 0, N, N, 2) = sigma_mx.transpose();

			sigma_.resize( N + 2, N + 2 );
			sigma_ = tmp_sigma;

			// add new aruco id
			aruco_ids_.push_back( it.aruco_id_);
		}
	}
}

void EKFSlam::showMarkers() const
{
	cv::Mat image = cv::Mat::zeros(600, 600, CV_8UC3);
	cv::circle( image, cv::Point( 300, 300 ), 3, cv::Scalar(0, 0, 255), -1 );
	for( int i = 4; i < x_.rows(); i += 2 ){
		double m_x = x_( i - 1, 0 );
		double m_y = x_( i, 0 );

		cv::circle( image, cv::Point(300 - m_y * 100, 300 - m_x * 100), 6, cv::Scalar(255, 0, 0), 1 );
	}	

	cv::imshow("map", image);
}

void EKFSlam::showMarkers( cv::Mat &image ) const
{
        cv::circle( image, cv::Point( 500, 300 ), 3, cv::Scalar(0, 0, 255), -1 );
        for( int i = 4; i < x_.rows(); i += 2 ){
                double m_x = x_( i - 1, 0 );
                double m_y = x_( i, 0 );
		std::cout<<"marker pose : ( "<<m_x<<", "<<m_y<<" )"<<std::endl;
		
                cv::circle( image, cv::Point(500 - m_y * 60, 300 - m_x * 60), 6, cv::Scalar(0, 255, 0), -1 );
        }

}

// 暴力搜素
bool EKFSlam::checkLandmark ( int aruco_id, int& landmark_idx )
{
    for(size_t i = 0; i < aruco_ids_.size(); i ++){
        if(aruco_id == aruco_ids_.at(i)){
            landmark_idx = i;
            return true;
        }
    }
    return false;
}

void EKFSlam::showMarkedImg() const
{
	cv::Mat markered_img = observe->getMarkeredImg();
	cv::imshow( "markered image", markered_img );
	
}

}
