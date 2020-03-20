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

	void ExtendedLambdaSetup() {
		estimator::pseudo_measurement::getOrCreatePenaltyMap()->addVisual(osg_viz::root);
		//osg_viz::addMapVisualizer(estimator::pseudo_measurement::SixdaysHeightMap::width*0.1,estimator::pseudo_measurement::SixdaysHeightMap::height*0.1);
		typedef zavi::functions::SixdaysMidwayFunctor functor;
		functor function=functor();
		typedef estimator::LambdaPathUKFExtended ESTIMATOR_TYPE;
		typedef estimator::XsensBatchWithBiasEstimator BATCHESTIMATOR;
		//zavi::osg_viz::root->addChild(zavi::osg_viz::functorGeometry(0,166.6,0.1,function));

		//std::shared_ptr<zavi::plugin::TrajectoryGeneratorPlugin> imu(new zavi::plugin::TrajectoryGeneratorPlugin(function,0.0004,pow(0.1*M_PI/180.,2),0.01,true,false, 1e-10, {1000,2000,1000,2000,1000,2000}));
		//estimator::pseudo_measurement::map.addPointsWithHeight(zavi::osg_viz::root,1.0,1e-2,2e-1);
		//zavi::osg_viz::root->addChild(zavi::osg_viz::functorGeometry(0.,168,0.1,function));
		libconfig::Setting & conf=cfg->lookup((const char *)cfg->lookup("trial"));
		LOG(INFO)<< "Running trial number: " << (int ) conf["trial_number"];
		const double skip_time=conf["skip_time"];

		std::shared_ptr<plugin::FileIMUPlugin> imu(new plugin::FileIMUPlugin(conf["imu_file"],skip_time,conf["end_time"],cfg->lookup("fast_forward_imu")));

		std::shared_ptr<ESTIMATOR_TYPE > ukf_estimator=ESTIMATOR_TYPE::createLambdaPathUKFExtended(imu, function);

		auto align_state=readTrialArray("alignment",16);
		ukf_estimator->setAlignment(Eigen::Map<Eigen::Matrix4d>(align_state.get()));
		ESTIMATOR_TYPE::STATE_TYPE start_state=ESTIMATOR_TYPE::STATE_TYPE::Zero();
		start_state(0,0)=0;
		start_state(1,0)=0;
		Eigen::Map<Eigen::Matrix3d>(start_state.block<9,1>(11,0).data())=Eigen::Vector3d(-1,-1,1).asDiagonal();
		//start_state.block<3,1>(18,0)=*(imu->getAngularVelocity());
		ESTIMATOR_TYPE::STATE_COV_TYPE start_cov;
		start_cov=start_cov.Identity()*1e-1;
		ukf_estimator->setStart(start_state,start_cov);
		//osg_viz::addStateVisualiser(ukf_estimator,osg::Vec4d(1,0,0,1));
		//osg_viz::ErrorEllipse::addErrorEllipse(ukf_estimator);
		//imu->addDataCallback(estimator::pseudo_measurement::ukfManifoldMeasurement<estimator::pseudo_measurement::FunctionOrientationMeasurement, ESTIMATOR_TYPE, functor>,ukf_estimator);
		imu->addDataCallback(estimator::pseudo_measurement::ukfMeasurement<estimator::pseudo_measurement::FunctionOrientationChangeMeasurement, ESTIMATOR_TYPE, functor>,ukf_estimator);//crashes for any reason
		imu->addDataCallback(estimator::pseudo_measurement::ukfMeasurement<estimator::pseudo_measurement::NoSideUpDriftConstrain, ESTIMATOR_TYPE>,ukf_estimator);

		std::shared_ptr<BATCHESTIMATOR> estimator=BATCHESTIMATOR::createXsensBatchWithBiasEstimator(imu,true,skip_time);
		auto ukfsaver =std::make_shared<estimator::callbacks::LambdaPathSmoothedReaderCallback<36, ESTIMATOR_TYPE > >();

		imu->addBatchCallback(ukf_estimator->smoothCallback,ukf_estimator);
		imu->addBatchCallback(ukfsaver->callback, ukf_estimator);
		imu->addBatchCallback(estimator->batchCallback,estimator);

		estimator->addConstraint(std::make_shared<estimator::pseudo_measurement::MeasurementConstraint< estimator::pseudo_measurement::NoSideUpDriftConstrain,BATCHESTIMATOR> >(), NULL);
		estimator->addConstraint(std::make_shared<estimator::pseudo_measurement::MeasurementConstraint< estimator::pseudo_measurement::PenaltyMapConstrain,BATCHESTIMATOR> >(), NULL);

		auto options=estimator->getOptions();
		options->minimizer_type=ceres::TRUST_REGION;
		//options->trust_region_strategy_type = ceres::DOGLEG;
		options->preconditioner_type=ceres::CLUSTER_JACOBI;
		options->linear_solver_type=ceres::SPARSE_NORMAL_CHOLESKY;
		options->dogleg_type=ceres::SUBSPACE_DOGLEG;
		options->minimizer_progress_to_stdout = true;
		options->num_threads=10;
		options->gradient_tolerance=1e-10;
		options->max_num_iterations=cfg->lookup("num_iterations");
		options->update_state_every_iteration=true;
		options->use_nonmonotonic_steps=true;
		options->check_gradients=false;

		//osg_viz::addStateVisualiser(estimator,osg::Vec4d(0,1,0,1));
		//osg_viz::addStateVisualiser(ukfsaver,osg::Vec4d(0,0,1,1));

		//set states of batch estimator
		estimator->setInitialStateArray(ukfsaver->callback->states);

		std::shared_ptr<zavi::osg_util::CameraToVideo> callback(new osg_util::CameraToVideo(osg_viz::viewer,"iteration"));
		estimator->addCallback(callback, estimator->CI_EVERY_SOLVER_ITERATION_CALLBACK);

		std::shared_ptr<zavi::osg_util::CameraToVideo> callback_replay(new osg_util::CameraToVideo(osg_viz::viewer,"replay",5));
		//estimator->addCallback(callback_replay,estimator->CI_REPLAY_COUNTER_INCREASE);
		std::shared_ptr<zavi::osg_util::CameraToVideo> callback_photo(new osg_util::CameraToVideo(osg_viz::viewer,"photo"));
		auto key_p=KeyCallbacker::createKeyCallbacker(&osg_viz::viewer,osgGA::GUIEventAdapter::KEY_P);
		key_p->addCallback(callback_photo,KeyCallbacker::CI_KEY);

		auto save_state=std::make_shared<estimator::callbacks::BatchStateToFileCallback<BATCHESTIMATOR> >(conf["batch_save_file"], conf["time_diff_save_file"], conf["input_mask_save_file"]);
		estimator->addCallback(save_state,estimator->CI_ESTIMATE_READY);

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
		double freq=100.;
		ThreadedObject::start_thread(imu,3*freq,RUNNING);
		ThreadedObject::start_thread(estimator,freq,RUNNING);
		ThreadedObject::start_thread(ukfsaver,freq,RUNNING);

	}

}

