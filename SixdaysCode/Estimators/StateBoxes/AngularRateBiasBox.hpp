/*
 * AccelerationBiasBox.hpp
 *
 *  Created on: 01.11.2018
 *      Author: tomlucas
 */

#ifndef ESTIMATORS_STATEBOXES_ANGULARRATEBIASBOX_HPP_
#define ESTIMATORS_STATEBOXES_ANGULARRATEBIASBOX_HPP_

#include "StateBox.hpp"
#include <eigen3/Eigen/Core>
#include "../../Plugins/imu_plugin.hpp"

namespace zavi
::estimator::state_boxes {
	/**
	 * Models the bias of a gyrometer as the sum of a constant and an exponentially decaying random variable
	 */
	class AngularRateBiasBox : public StateBox<6,6,0> {
	private:
		Eigen::Matrix<double,3,1> bias_correlations; //!< correlations for the random variable
		Eigen::Matrix<double, 6,1> bias_noises; //!< noise of the biases (0 for the constant)
	public:
		AngularRateBiasBox(std::shared_ptr<plugin::SensorPlugin> sensor):StateBox<outer_size,inner_size,input_size>(sensor) {
			zavi::plugin::IMU_Plugin * imu=static_cast<zavi::plugin::IMU_Plugin *>(sensor.get());
			bias_correlations= imu->getAngularRateBiasCorr().array().inverse();
			bias_noises=bias_noises.Zero();
			bias_noises.block<3,1>(3,0)=imu->getAngularRateBiasCov().diagonal().cwiseSqrt();
			assert_inputs(bias_noises,bias_correlations);
		}

		template<typename T>
		static inline OUTER_T<T> boxPlus(const OUTER_T<T> &state,const INNER_T<T> &delta) {
			assert_inputs(state,delta);
			return state+delta;
		}
		template<typename T>
		static inline INNER_T<T> boxMinus(const OUTER_T<T> &a,const OUTER_T<T> &b) {
			assert_inputs(a,b);
			return b-a;
		}


		template<typename T>
		inline OUTER_T<T> stateTransition(const OUTER_T<T> & state,double time_diff ) {
			assert_inputs(state,time_diff);
			OUTER_T<T> result(state);
			result.template block<3,1>(3,0)=(result.template block<3,1>(3,0).array()-result.template block<3,1>(3,0).array()*bias_correlations.array()*time_diff );
			return result;
		}
		inline INNER_T<double> getSTD(double time_diff) {
			return bias_noises*time_diff;
		}
	};
}

#endif /* ESTIMATORS_STATEBOXES_ANGULARRATEBIASBOX_HPP_ */
