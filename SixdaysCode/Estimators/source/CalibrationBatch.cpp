/*
 * XsensBatch.cpp
 *
 *  Created on: 06.02.2019
 *      Author: tomlucas
 */

#include "../CalibrationBatch.hpp"
#include <tuple>
#include <ceres/internal/variadic_evaluate.h>
#include "../StateBoxes/OrientationSpeedDerivBox.hpp"
#include "../PseudoMeasurements.hpp"
namespace zavi
::estimator {
	Eigen::Matrix<double,9,1> ident=Eigen::Map<Eigen::Matrix<double,9,1> >(Eigen::Matrix3d(Eigen::Matrix3d::Identity()).data());
	CalibrationBatchEstimator::CalibrationBatchEstimator(std::shared_ptr<plugin::IMU_Plugin> imu,bool output_replay, double skip_time): BatchEstimator<zavi::estimator::model::CalibrationModel>(imu,output_replay,skip_time) {
		addConstraint(transitionConstrain,NULL);
		(std::get<state_boxes::OrientationSpeedDerivBox>(box_model.boxes)).std=state_boxes::OrientationSpeedDerivBox::INNER_T<double>::Ones()*0.01;

		addConstraint(stateConstrain<Eigen::Matrix<double,6,1> ,estimator::model::CalibrationModel::input_measurement,6>, &advanced_inputs);
		addConstraint(singleStateConstrain<model::CalibrationModel::orientation_measurement,9,0,ident>, NULL);
	}

	std::shared_ptr<CalibrationBatchEstimator> CalibrationBatchEstimator::createCalibrationBatchEstimator(std::shared_ptr<plugin::IMU_Plugin> imu, bool output_replay, double skip_time) {
		std::shared_ptr<CalibrationBatchEstimator> estimator(new CalibrationBatchEstimator(imu,output_replay,skip_time));
		imu->addDataCallback(imuCallback,estimator);     //< add the basic callback from batchestimator
		return estimator;
	}

	void CalibrationBatchEstimator::setInitialState(double * data,unsigned int index) {
		memset(data,0,sizeof(double)*MODEL_TYPE::outer_size);
		static Eigen::Matrix3d ident=ident.Identity();
		memcpy(&data[0],ident.data(),sizeof(double)*9);

	}

	void CalibrationBatchEstimator::imuCallback(plugin::SensorPlugin * plug,
			void* estimator, double time) {
		static double last_time=time-0.0001;     // fix for zero start time
		double time_diff=time-last_time;
		assert(time_diff > 0.);
		assert(time_diff !=NAN);
		CalibrationBatchEstimator * esti = static_cast<CalibrationBatchEstimator *>(estimator);
		plugin::IMU_Plugin *imu = static_cast<plugin::IMU_Plugin*>(plug);
		Eigen::Matrix<double,6,1> input=input.Zero();
		input.block<3,1>(0,0)=imu->getAcceleration();
		input.block<3,1>(3,0)=imu->getAngularVelocity();
		esti->advanced_inputs.push_back(input);
		esti->time_diffs.push_back(time_diff);
		last_time=time;
	}

	void CalibrationBatchEstimator::advanceState(double * start, double * next,double time_diff, double next_index) {
		Eigen::Map<STATE_TYPE_T<double>> map_next(next);
		map_next=box_model.stateTransitionFunction<double>(Eigen::Map<STATE_TYPE_T<double>>(start),time_diff);
		map_next.block<3,1>(6,0)=box_model.getOrientation<double>(map_next)*inputs[next_index].block<3,1>(0,0)-zavi::plugin::IMU_Plugin::gravity;
		map_next.block<3,1>(18,0)=inputs[next_index].block<3,1>(3,0);
	}

	void CalibrationBatchEstimator::estimate() {
		printf("IsCalled");
		int num_states=advanced_inputs.size();
		states=(double *)malloc(num_states*MODEL_TYPE::outer_size*sizeof(double));
		ceres::Solver::Options options;
		options.minimizer_type=ceres::TRUST_REGION;
		options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
		options.linear_solver_type=ceres::SPARSE_NORMAL_CHOLESKY;
		options.dogleg_type=ceres::SUBSPACE_DOGLEG;
		options.minimizer_progress_to_stdout = true;
		options.num_threads=10;
		options.gradient_tolerance=1e-10;
		options.max_num_iterations=1000;
		options.update_state_every_iteration=true;
		options.use_nonmonotonic_steps=false;
		ceres::Solver::Summary summary;

		osg::ref_ptr<osg::Geode> geo(new osg::Geode());
		zavi::osg_viz::root->addChild(geo);
		int state_size=num_states;

		ceres::LocalParameterization * local_parameterization=new ceres::AutoDiffLocalParameterization<MODEL_TYPE,MODEL_TYPE::outer_size,MODEL_TYPE::inner_size>(new MODEL_TYPE(box_model));

		ceres::Problem problem;
		auto align_state= readTrialArray("alignment",16);
		addAlignment(problem,align_state.get(),false);

		/*double align_state[16]= {0.999857 , -0.011412 ,-0.0125132 , 0.075947,
		 0.0101525 , 0.995284, -0.0964706 , 0.135885,
		 0.0135551 , 0.0963298 , 0.995257 , 0.874725 ,
		 0 , 0 , 0 , 1};*/

		/*
		 0.999905  -0.0101993 -0.00935916   0.0725492
		 0.0107625    0.998003   0.0622406 -0.00366175
		 0.00870565  -0.0623353    0.998017    0.885382
		 0           0           0           1

		 */
		/*0.982053  0.001159  0.188604  -0.14286
		 0.026258  0.989402 -0.142804  0.079499
		 -0.18677  0.145194  0.971615  0.898037
		 0         0         0         1*/
		printf(Eigen::Map<ALIGNMENT_TYPE<double>>(alignment));

		addConstraints(problem,states,state_size,this);

		for(int i=0; i < state_size; i++) {
			setInitialState(&states[i*MODEL_TYPE::outer_size],i);
			problem.SetParameterization(&states[i*MODEL_TYPE::outer_size],local_parameterization);
			for(int k=0; k < MODEL_TYPE::outer_size;k++) {
				problem.SetParameterLowerBound(&states[i*MODEL_TYPE::outer_size], k, lower_bound(k,0));
				problem.SetParameterUpperBound(&states[i*MODEL_TYPE::outer_size], k, upper_bound(k,0));

			}
		}

		printf("Starting solver\n");
		ceres::Solve(options, &problem, &summary);
		std::cout << summary.FullReport() << "\n";

		std::vector<STATE_TYPE_T<double>> all_states;
		for(int i=0; i < state_size; i++) {
			all_states.push_back(STATE_TYPE_T<double>(&states[i*MODEL_TYPE::outer_size]));
		}
		printf("Bias is ");
		printf(Eigen::Map<ALIGNMENT_TYPE<double>>(alignment).transpose());
		printf(eigen_util::inverseEulerRodriguez<double>(Eigen::Map<ALIGNMENT_TYPE<double>>(alignment).block<3,3>(0,0)));
		printf(Eigen::Map<ALIGNMENT_TYPE<double>>(alignment).block<3,1>(0,3));
		printf("Estimate complete\n");
		//zavi::plot::stateDraw<0>(states,"Batch estimated states");
		state_vector=STATE_TYPE_T
		<double>(&states[(state_size-1)*MODEL_TYPE::outer_size]);
		delete states;
		notifyCallbacks(CI_ESTIMATE_SUBSEQUENCE_READY);
		printf("Completed Estimation");
	}

}
