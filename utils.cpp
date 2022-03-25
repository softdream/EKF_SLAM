#include "utils.h"


namespace slam
{

cv::FileStorage Utils::fs;

bool Utils::openYamlFile( const std::string &file_name )
{
	fs.open( file_name, cv::FileStorage::READ );
	
	if( !fs.isOpened() ) {
                std::cerr<<"open yaml failed ..."<<std::endl;
                return false;
        }

	return true;
}

void Utils::closeYamlFile() 
{
	return fs.release();
}

void Utils::normalizeAngle( double &angle )
{
	if( angle >= PI ){
		angle -= TWO_PI;
	}
	
	if( angle < -PI){
		angle += TWO_PI;
	}
}


}
