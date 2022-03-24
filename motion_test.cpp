#include "motion.h"
#include "utils.h"

int main()
{
	std::cout<<"================== Motion TEST ================="<<std::endl;
	double last_enl = 0;
 	double last_enr = 0;

	bool isInitilized = false;

	slam::Motion::openASimulationFile( "encoder_data.txt" );

	std::vector<slam::Motion> motionsVec;
	
	//while( !slam::Motion::endOfFile() ){
	while( slam::Motion::getFrameCount() < 2000 ){
		double enl = 0;
		double enr = 0;
		int frame = slam::Motion::readAFrameData( enl, enr );	
		std::cout<<"frame: "<<frame<<", enl = "<<enl<<", enr = "<<enr<<std::endl;
	
		if( !isInitilized ){
			last_enl = enl;
			last_enr = enr;
			isInitilized = true;
			
			continue;
		}

		slam::Motion motion_data = slam::Motion::getMotionValueFromEncoder( enl, enr, last_enl, last_enr );	
		motionsVec.push_back( motion_data );

		last_enl = enl;
		last_enr = enr;
	}

	//for( auto it : motionsVec ){
		//std::cout<<"it.delta_s = "<<it.delta_s_<<", it.delta_theta = "<<it.delta_theta_<<std::endl;
	//}

	slam::Motion::showTrajectory( motionsVec );

	slam::Motion::closeSimulationFile();
	return 0;
}
