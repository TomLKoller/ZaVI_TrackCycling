/*
 * NOBatchEstimator.hpp
 *
 *  Created on: 04.09.2018
 *      Author: tomlucas
 */

#ifndef ESTIMATORS_CALIBRATIONBATCH_HPP_
#define ESTIMATORS_CALIBRATIONBATCH_HPP_

#include "BatchEstimator.hpp"
#include "Models/CalibrationModel.hpp"

#include "../Plugins/imu_plugin.hpp"
namespace zavi
::estimator {

	/**
	 * No Orientation BatchEstimator takes use of NoOrientationModel and the BatchEstimator
	 */
	class CalibrationBatchEstimator: public BatchEstimator <model::CalibrationModel> {
		static inline Eigen::Matrix<double,6,1> start_pose_velocity;
	protected:
		CalibrationBatchEstimator(std::shared_ptr<plugin::IMU_Plugin> imu,bool output_replay=false, double skip_time=0);
	public:
		virtual ~CalibrationBatchEstimator(){
		}
		/**
		 * Create an NOBatch_Estimator shared pointer
		 * @param imu the imu to read the basic data
		 * @return shared pointer to the estimator
		 */
		static std::shared_ptr<CalibrationBatchEstimator> createCalibrationBatchEstimator(std::shared_ptr<plugin::IMU_Plugin> imu, bool output_replay=false, double skip_time=0) ;


		/**
		 * Set the initial values for a batch states
		 * @param data  the data for the current state (size MODEL_TYPE::outer_size)
		 * @param index the index of the state (the pointer points to the current state but if you have to know, which inputs you want to assign)
		 */
		virtual void setInitialState(double * data,unsigned int index) ;



		/**
		 *
		 * @param start start point for transition
		 * @param next  after the transition
		 * @param time_diff time_diff between points
		 * @param next_index the index to take readings of input
		 */
		virtual void advanceState(double * start, double * next,double time_diff, double next_index);

		/**
		 * Callback for imu
		 * @param plug the imu plugin which has new data
		 * @param estimator the estimator which gets the new data
		 * @param time the current time stamp
		 */
		static void imuCallback(plugin::SensorPlugin * plug,
				void* estimator, double time);


		virtual void estimate();

	public:
		std::vector<Eigen::Matrix<double,6,1> > advanced_inputs;
	private:
	};

}
//zavi::estimator

#endif /* ESTIMATORS_CALIBRATIONBATCH_HPP_ */
