
#include <ZaviConfig.hpp>
#include "OSG_VIZ/osg_viz.hpp"
#include <ZaVI_Utils.hpp>
#include "ODESetups.hpp"
#include "Setups/Functors.hpp"
#include <glog/logging.h>
#define LOOP_FREQUENCY 100
#define TIME_LAPSE 1


#define Ente

int main(int argc, char** argv) {
	printf("Starting up");
	initConfig("Config/main_config.cfg");
	google::InitGoogleLogging(argv[0]);
	zavi::Timer::initTime(cfg->lookup("time_lapse"));
	zavi::osg_viz::initOSG();

	zavi::setups::ExtendedLambdaSetup();

	zavi::printf("starting osg thread\n");
	zavi::osg_viz::osgStart();
	zavi::ThreadedObject::joinAll();

	return 0;
}

