#include "SetupBase.hpp"
#include "../Estimators/PseudoMeasurementsHelperClasses/PenaltyMap.hpp"
#include "../Estimators/PseudoMeasurements.hpp"
#include "../Estimators/XsensBatchWithBias.hpp"
#include "../Plugins/FileIMUPlugin.hpp"
#include <CameraToVideo.hpp>

#include "../Estimators/Callbacks/LambdaPathStateReaderCallback.hpp"

#include "../Estimators/LambdaPathUKFExtended.hpp"

#include "Functors.hpp"
#include "../Sixdays/LaserBarrier.hpp"
#include <KeyCallbacker.hpp>
#include "../Estimators/Callbacks/BatchStateToFile.hpp"
namespace zavi
::setups {

	void LoadResultSetup() {
		estimator::pseudo_measurement::getOrCreatePenaltyMap()->addVisual(osg_viz::root);
		//osg_viz::addMapVisualizer(estimator::pseudo_measurement::SixdaysHeightMap::width*0.1,estimator::pseudo_measurement::SixdaysHeightMap::height*0.1);
		typedef zavi::functions::SixdaysMidwayFunctor functor;
		functor function=functor();
		typedef estimator::LambdaPathUKFExtended ESTIMATOR_TYPE;
		typedef estimator::XsensBatchWithBiasEstimator BATCHESTIMATOR;
		//zavi::osg_viz::root->addChild(zavi::osg_viz::functorGeometry(0,170.,0.1,function));

		//std::shared_ptr<zavi::plugin::TrajectoryGeneratorPlugin> imu(new zavi::plugin::TrajectoryGeneratorPlugin(function,0.0004,pow(0.1*M_PI/180.,2),0.01,true,false, 1e-10, {1000,2000,1000,2000,1000,2000}));
		//estimator::pseudo_measurement::map.addPointsWithHeight(zavi::osg_viz::root,1.0,1e-2,2e-1);
		//zavi::osg_viz::root->addChild(zavi::osg_viz::functorGeometry(0.,168,0.1,function));
		libconfig::Setting & conf=cfg->lookup((const char *)cfg->lookup("trial"));
		LOG(INFO)<< "Running trial number: " << (int ) conf["trial_number"];
		const double skip_time=conf["skip_time"];

		std::shared_ptr<plugin::FileIMUPlugin> imu(new plugin::FileIMUPlugin(conf["imu_file"],skip_time,conf["end_time"],cfg->lookup("fast_forward_imu")));

		std::shared_ptr<BATCHESTIMATOR> estimator=BATCHESTIMATOR::createXsensBatchWithBiasEstimator(imu,true,skip_time);

		//osg_viz::addStateVisualiser(estimator,osg::Vec4d(1,0,0,1),osg_viz::root,true);
		std::shared_ptr<zavi::osg_util::CameraToVideo> callback_replay(new osg_util::CameraToVideo(osg_viz::viewer,"replay",5));
		//estimator->addCallback(callback_replay,estimator->CI_REPLAY_COUNTER_INCREASE);
		std::shared_ptr<zavi::osg_util::CameraToVideo> callback_photo(new osg_util::CameraToVideo(osg_viz::viewer,"photo"));
		auto key_p=KeyCallbacker::createKeyCallbacker(&osg_viz::viewer,osgGA::GUIEventAdapter::KEY_P);
		key_p->addCallback(callback_photo,KeyCallbacker::CI_KEY);

		Eigen::Vector3d start_line_offset(-2.2,0,0);
		Eigen::Vector3d offset1(28.536,30.1,0);
		Eigen::Vector3d offset2(25.086,7,0);
		offset1+=start_line_offset;
		offset2-=start_line_offset;

		auto laser1=sixdays::LaserBarrier::createLaserBarrier(1,offset1+Eigen::Vector3d(6.9,0,0),offset1+Eigen::Vector3d(0,6.8,2),conf["laser1"],zavi::osg_viz::root);
		auto laser2=sixdays::LaserBarrier::createLaserBarrier(2,offset1+Eigen::Vector3d(0,0,0),offset1+Eigen::Vector3d(6.9,6.8,1.7),conf["laser2"],zavi::osg_viz::root);
		auto laser3=sixdays::LaserBarrier::createLaserBarrier(3,offset1+Eigen::Vector3d(6.9+16.71,0,0),offset1+Eigen::Vector3d(6.9+16.71,6.8,2.09),conf["laser3"],zavi::osg_viz::root);

		auto laser4=sixdays::LaserBarrier::createLaserBarrier(4,offset2+Eigen::Vector3d(6.8,0,0),offset2+ Eigen::Vector3d(0,-6.8,2.09),conf["laser4"], zavi::osg_viz::root);
		auto laser5=sixdays::LaserBarrier::createLaserBarrier(5,offset2+Eigen::Vector3d(0,0,0), offset2+Eigen::Vector3d(6.8,-6.8,1.73),conf["laser5"],zavi::osg_viz::root);
		auto laser6=sixdays::LaserBarrier::createLaserBarrier(6,offset2+Eigen::Vector3d(6.8+17.04,0,0),offset2+ Eigen::Vector3d(6.8+17.04,-6.8,2.1),conf["laser6"],zavi::osg_viz::root);

		estimator->addCallback(laser1,estimator->CI_FAST_REPLAY_ITERATION);
		estimator->addCallback(laser2,estimator->CI_FAST_REPLAY_ITERATION);
		estimator->addCallback(laser3,estimator->CI_FAST_REPLAY_ITERATION);
		estimator->addCallback(laser4,estimator->CI_FAST_REPLAY_ITERATION);
		estimator->addCallback(laser5,estimator->CI_FAST_REPLAY_ITERATION);
		estimator->addCallback(laser6,estimator->CI_FAST_REPLAY_ITERATION);

		//start everything
		double freq=cfg->lookup("thread_freq");
		std::ifstream input_file((const char * ) conf["batch_save_file"], std::ios::binary);
		std::ifstream time_diffs((const char * ) conf["time_diff_save_file"], std::ios::binary);
		std::ifstream input_mask((const char * ) conf["input_mask_save_file"], std::ios::binary);
		std::vector<char> buffer(std::istreambuf_iterator<char>(input_file), {});
		std::vector<char> t_buffer(std::istreambuf_iterator<char>(time_diffs), {});
		std::vector<char> i_buffer(std::istreambuf_iterator<char>(input_mask), {});
		estimator->replay(buffer.data(),t_buffer.data(),i_buffer.data(),buffer.size()/sizeof(double)*sizeof(char));
		ThreadedObject::start_thread(estimator,freq,RUNNING);
	}

}

