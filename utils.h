#ifndef __UTILS_H
#define __UTILS_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include"opencv2/core/eigen.hpp"

#include <opencv2/opencv.hpp>

#include <iostream>

#include <iomanip>

#include <type_traits>

namespace slam
{

template<typename T, int Rows, int Cols, 
	template<typename U, int R, int C, int Option, int MaxR, int MaxC> 
	class EigenType>
struct is_Eigen_type
{
	static const bool value = false;
};

template<typename T, int Rows, int Cols>
struct is_Eigen_type<T, Rows, Cols, Eigen::Matrix>
{
	using type = Eigen::Matrix<T, Rows, Cols>;
	static const bool value = true;
};


class Utils
{
public:
	static bool openYamlFile( const std::string &file_name );
		
	template<typename T>
	static bool readParameter( const std::string &parameter_type, 
				   const std::string &parameter_name,
				   T &parameter)
	{
		std::cout<<std::setprecision(20);

		cv::FileNode node = fs[parameter_type];
		if( node.empty() ){
			return false;
		}

		parameter = (T)( node[parameter_name] );
		//std::cout<<parameter_name<<" : " << parameter<<std::endl;		
		return true;
	}	

	template<typename T, 
		typename = typename std::enable_if<is_Eigen_type<typename T::value_type, T::RowsAtCompileTime, T::ColsAtCompileTime, Eigen::Matrix>::value>::type>
	static bool readParameterEigen( const std::string &parameter_name, T &parameter )
	{
		cv::Mat matrix;	
		//std::cout<<"parameter.cols = "<<parameter.cols()<<std::endl;
		fs["Trc"] >> matrix;
		//std::cout<<"Trc = "<<std::endl<<matrix<<std::endl;	
		cv::cv2eigen(matrix, parameter); // cv::eigen2cv();
	}

	static void closeYamlFile();
	

	static void normalizeAngle( double &angle );

public:
	constexpr static double PI = 3.141592653;	
	constexpr static double TWO_PI = 2.0 * PI;
private: 
	static cv::FileStorage fs;
};

}

#endif
