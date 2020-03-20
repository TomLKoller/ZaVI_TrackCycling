/*
 * XsensBatch.cpp
 *
 *  Created on: 06.02.2019
 *      Author: tomlucas
 */

#include "../XsensBatchWithBias.hpp"
#include <tuple>
#include <ceres/internal/variadic_evaluate.h>
#include "../Callbacks/BatchStateToFile.hpp"
#include "../../Sixdays/LaserBarrier.hpp"
namespace zavi
::estimator {
	XsensBatchWithBiasEstimator::XsensBatchWithBiasEstimator(std::shared_ptr<plugin::IMU_Plugin> imu,bool output_replay, double skip_time): BatchEstimator<zavi::estimator::model::XsensModelWithBias>(imu,output_replay,skip_time),initial_states(NULL) {
		addConstraint(transitionConstrain,NULL);
		addConstraint(inputConstrain,&inputs);
		//addConstraint(stateConstrain<Eigen::Matrix<double,9,1>,MODEL_TYPE::orientation_measurement,9 >,&orientations);
		start_pose_velocity=start_pose_velocity.Zero();
		start_pose_velocity(0,0)=15;
		start_pose_velocity(1,0)=25;
		//addConstraint(singleStateConstrain<MODEL_TYPE::pos_velocity_measurement,6,0,start_pose_velocity>,NULL);
		// x y bounds
		/*lower_bound(0,0)=0;
		 upper_bound(0,0)=75.9;
		 lower_bound(1,0)=0;
		 upper_bound(1,0)=36.9;
		 lower_bound(2,0)=0.;
		 upper_bound(2,0)=6.;*/
		(std::get<state_boxes::OrientationSpeedDerivBox>(box_model.boxes)).std=state_boxes::OrientationSpeedDerivBox::INNER_T<double>::Ones()*0.1;
		state_boxes::CartesianBox::INNER_T<double > std=std.Zero();
		std(0)=std(1)=std(2)=0.1;
		std (3)=std (4)=std (5)=0.1;
		std(6)=std(7)=std(8)=100;
		std::get<state_boxes::CartesianBox>(box_model.boxes).std=std;
	}

	std::shared_ptr<XsensBatchWithBiasEstimator> XsensBatchWithBiasEstimator::createXsensBatchWithBiasEstimator(std::shared_ptr<plugin::IMU_Plugin> imu, bool output_replay, double skip_time) {
		std::shared_ptr<XsensBatchWithBiasEstimator> estimator(new XsensBatchWithBiasEstimator(imu,output_replay,skip_time));
		imu->addDataCallback(imuCallback,estimator);     //< add the basic callback from batchestimator
		return estimator;
	}

	void XsensBatchWithBiasEstimator::setInitialState(double * data,unsigned int index) {
		if(initial_states!=NULL and initial_states->size() > index) {
			memcpy(data,&(initial_states->data()[index]),sizeof(double)*MODEL_TYPE::outer_size);
			return;
		}

		memset(data,0,sizeof(double)*MODEL_TYPE::outer_size);
		Eigen::Vector3d world_acceleration=(Eigen::Map<Eigen::Matrix<double,3,3> >(orientations[index].data())*inputs[index].block<3,1>(0,0))-zavi::plugin::IMU_Plugin::gravity;
		/*data[0]=15;
		 data[1]=25;
		 data[2]=1;*/
		setNeutralPose(data);
		memcpy(&data[6],world_acceleration.data(),sizeof(double)*3);     //acceleration
		memcpy(&data[9],orientations[index].data(),sizeof(double)*9);
		memcpy(&data[18],inputs[index].block<3,1>(3,0).data(),sizeof(double)*3);//angular velocity

	}

	void XsensBatchWithBiasEstimator::imuCallback(plugin::SensorPlugin * plug,
			void* estimator, double time) {
		static double last_time=time-0.0001;     // fix for zero start time
		double time_diff=time-last_time;
		assert(time_diff > 0.);
		assert(time_diff !=NAN);
		XsensBatchWithBiasEstimator * esti = static_cast<XsensBatchWithBiasEstimator *>(estimator);
		plugin::IMU_Plugin *imu = static_cast<plugin::IMU_Plugin*>(plug);
		MODEL_TYPE::INPUT_T<double> input=MODEL_TYPE::INPUT_T<double>::Zero();
		input.block<3,1>(0,0)=imu->getAcceleration();
		input.block<3,1>(3,0)=imu->getAngularVelocity();
		esti->orientations.push_back(Eigen::Matrix<double,9,1>(imu->getOrientation().data()));
		esti->inputs.push_back(input);
		esti->time_diffs.push_back(time_diff);
		esti->input_mask.push_back(imu->isDataValid());
		last_time=time;
	}

	void XsensBatchWithBiasEstimator::advanceState(double * start, double * next,double time_diff, double next_index) {
		Eigen::Map<STATE_TYPE_T<double>> map_next(next);
		map_next=box_model.stateTransitionFunction<double>(Eigen::Map<STATE_TYPE_T<double>>(start),time_diff);
		map_next.block<3,1>(6,0)=box_model.getOrientation<double>(map_next)*inputs[next_index].block<3,1>(0,0)-zavi::plugin::IMU_Plugin::gravity;
		map_next.block<3,1>(18,0)=inputs[next_index].block<3,1>(3,0);
	}

	void XsensBatchWithBiasEstimator::replay(void * data, void * time_diffs_pointer, void * input_mask_pointer, size_t size) {
		if(data!=NULL) {
			current_estimation=(double *)malloc(size*sizeof(double));
			memcpy((void *) current_estimation,data,size*sizeof(double));
			time_diffs=std::vector<double>((double *) time_diffs_pointer,((double *) time_diffs_pointer) +size/MODEL_TYPE::outer_size);
			input_mask=std::vector<char>((char *) input_mask_pointer,((char *) input_mask_pointer) +size/MODEL_TYPE::outer_size/sizeof(char)*sizeof(char));
			LOG(INFO) << "size of input mask" << input_mask.size();
			fastReplay(size/MODEL_TYPE::outer_size);
			current_size=size/MODEL_TYPE::outer_size;
			osg::ref_ptr<osg::Geode> geo(new osg::Geode());
			zavi::osg_viz::root->addChild(geo);
			auto colors=sixdays::LaserBarrier::getVerticeColoring(current_size);
			if(cfg->lookup("show_only_dummy")) {
				int num_dummies=0;
				for(int i=0; i <input_mask.size();i++) {
					if(input_mask[i]) {
						colors->at(i)=osg::Vec4f(0,0,0,0);
					}
					else {
						num_dummies++;
					}
				}
				LOG(INFO) << "number of dummy states "<<num_dummies;

			}
			std::vector<Eigen::Vector3d> expected_accelerations;
			std::vector<STATE_TYPE_T<double> > states=std::vector<STATE_TYPE_T<double>>((STATE_TYPE_T<double> *) data,((STATE_TYPE_T<double> *) data) +size/MODEL_TYPE::outer_size);
			auto align_state= readTrialArray("alignment",16);
			for(auto state: states) {
				expected_accelerations.push_back(box_model.acceleration_measurement<double>(state,Eigen::Map<const ALIGNMENT_TYPE<double> >(align_state.get()),NULL));
			}
			callbacks::BatchStateToFileCallback<XsensBatchWithBiasEstimator>::storeData("expected_acc",expected_accelerations.data(),expected_accelerations.size()*3*sizeof(double));
			geo->addChild(zavi::osg_viz::pathShower((double *) current_estimation,colors, current_size,MODEL_TYPE::outer_size));
		}
	}

	void XsensBatchWithBiasEstimator::estimate() {
		if(inputs.size()==0)
		return;     // nothing to do if no no constraints can be put
		/*auto plt=plot::createPlot();
		 plot::drawLine(plt,time_diffs,"time_diffs");
		 plot::stateDraw<0>(inputs,"inputs",0);
		 */
		double * temp=NULL;

		num_states=inputs.size();
		states=(double *)malloc(num_states*MODEL_TYPE::outer_size*sizeof(double));

		IterationCounter counter(states,num_states,this);
		options.callbacks.push_back(&counter);
		ceres::Solver::Summary summary;

		osg::ref_ptr<osg::Geode> geo(new osg::Geode());
		zavi::osg_viz::root->addChild(geo);
		unsigned int state_size=num_states;

		ceres::LocalParameterization * local_parameterization=new ceres::AutoDiffLocalParameterization<MODEL_TYPE,MODEL_TYPE::outer_size,MODEL_TYPE::inner_size>(new MODEL_TYPE(box_model));

		ceres::Problem problem;
		auto align_state= readTrialArray("alignment",16);
		addAlignment(problem,align_state.get());

		/*
		 0.999905  -0.0101993 -0.00935916   0.0725492
		 0.0107625    0.998003   0.0622406 -0.00366175
		 0.00870565  -0.0623353    0.998017    0.885382
		 0           0           0           1

		 */

		printf(Eigen::Map<ALIGNMENT_TYPE<double>>(alignment));

		for(unsigned int i=0; i < state_size; i++) {
			setInitialState(&states[i*MODEL_TYPE::outer_size],i);
			problem.AddParameterBlock(&states[i*MODEL_TYPE::outer_size],MODEL_TYPE::outer_size,local_parameterization);
			for(int k=0; k < MODEL_TYPE::outer_size;k++) {
		    	problem.SetParameterLowerBound(&states[i*MODEL_TYPE::outer_size], k, lower_bound(k,0));
				problem.SetParameterUpperBound(&states[i*MODEL_TYPE::outer_size], k, upper_bound(k,0));

			}
		}
		addConstraints(problem,states,state_size,this);

		printf("Starting solver\n");
		ceres::Solve(options, &problem, &summary);
		std::cout << summary.FullReport() << "\n" << summary.message <<"\n";
		printf("Estimate complete\n");
		//zavi::plot::stateDraw<0>(states,"Batch estimated states");
		if(output_replay) {
			temp=(double *)current_estimation;     // safe pointer to delete old data
			current_estimation=(double *)malloc(state_size*MODEL_TYPE::outer_size*sizeof(double));
			memcpy((void *) current_estimation, states,state_size*MODEL_TYPE::outer_size*sizeof(double));
			fastReplay(state_size);
			current_size=state_size;

			if (temp!=NULL and temp !=states)
			delete temp;
			std::vector<STATE_TYPE_T<double>> all_states;
			std::vector<double> norms;
			auto input_measurement=typename MODEL_TYPE::input_measurement();
			double max_norm =0.;
			for(unsigned int i=0; i < state_size; i++) {
				all_states.push_back(STATE_TYPE_T<double>(&states[i*MODEL_TYPE::outer_size]));
				//norms.push_back((input_measurement.template operator()<double>(all_states.back(),Eigen::Map<Eigen::Matrix3d>(bias),NULL)-inputs[i]).norm());
				norms.push_back(MODEL_TYPE::getVelocity(all_states.back()).norm());
				if(norms.back() > max_norm)
				max_norm=norms.back();
			}

			for(int i=0; i < state_size; i++) {
				norms[i]=norms[i]/max_norm;
			}
			printf("Bias is ");
			printf(eigen_util::inverseEulerRodriguez<double>(Eigen::Map<ALIGNMENT_TYPE<double>>(alignment).block<3,3>(0,0)));
			zavi::printf("First state is :");
			zavi::printf(all_states[0]);

			if(geo->getNumChildren() >0) {
				geo->removeChild(0,1);
			}
			//geo->addChild(zavi::osg_viz::pathShower((double *) current_estimation,&norms[0], current_size,MODEL_TYPE::outer_size));
			DLOG(INFO)<<" Now building path visual for batch estimator";
			geo->addChild(zavi::osg_viz::pathShower((double *) current_estimation,sixdays::LaserBarrier::getVerticeColoring(state_size), current_size,MODEL_TYPE::outer_size));
			//zavi::plot::stateDraw<0>(all_states,"Estimated States");
			//zavi::plot::stateDraw<1>(inputs,"INputs");
		}
		else {
			state_vector=STATE_TYPE_T
			<double>(&states[(state_size-1)*MODEL_TYPE::outer_size]);
			delete states;
		}
		notifyCallbacks(CI_ESTIMATE_SUBSEQUENCE_READY);
		printf("Completed Estimation");
		notifyCallbacks(CI_ESTIMATE_READY);
	}

}
