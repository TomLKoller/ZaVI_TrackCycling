/*
 * OrientationBox.hpp
 *
 *  Created on: 01.11.2018
 *      Author: tomlucas
 */

#ifndef ESTIMATORS_STATEBOXES_HOMOCOORDBOX_HPP_
#define ESTIMATORS_STATEBOXES_HOMOCOORDBOX_HPP_

#include "StateBox.hpp"
#include <eigen3/Eigen/Core>
#include <Eigen_Utils.hpp>
#include <stdio.h>

namespace zavi
::estimator::state_boxes {
	/**
	 * A Box which contains homogenous coordinates
	 * This is basically the calibration matrix for the offset between IMU and body coordinates
	 */
	class HomoCoordBox : public StateBox<16,6,0> {
		template<typename T>
		using ROT_MATRIX= Eigen::Matrix<T,3,3>;


		template<typename T>
		inline static const Eigen::Matrix<T,4,4> remap(const OUTER_T<T> & state){
			return Eigen::Map<const Eigen::Matrix<T,4,4>>(state.data());
		}


	public:
		HomoCoordBox(std::shared_ptr<plugin::SensorPlugin> sensor):StateBox<outer_size,inner_size,input_size>(sensor) {
		}
		virtual ~HomoCoordBox() {};

		template<typename T>
		inline static OUTER_T<T> boxPlus(const OUTER_T<T> &state,const INNER_T<T> &delta) {
			assert_inputs(state,delta);
			Eigen::Matrix<T,4,4> result=result.Identity();
			result.template block<3,3>(0,0)=zavi::eigen_util::boxPlusOrientation<T>(remap(state).template block<3,3>(0,0),delta.template block<3,1>(0,0));
			result.template block<3,1>(0,3)=state.template block<3,1>(12,0)+delta.template block<3,1>(3,0);
			return Eigen::Map<OUTER_T<T>>(result.data());
		}
		template<typename T>
		inline static INNER_T<T> boxPlusInnerSpace(const INNER_T<T> &delta1,const INNER_T<T> &delta2) {
			assert_inputs(delta1,delta2);
			return delta1+delta2;
		}

		template<typename T>
		inline static INNER_T<T> boxMinus(const OUTER_T<T> &a,const OUTER_T<T> &b) {
			assert_inputs(a,b);
			INNER_T<T> result=result.Zero();
			result.template block<3,1>(0,0)=zavi::eigen_util::boxMinusOrientation<T,T>(remap(a).template block<3,3>(0,0),remap(b).template block<3,3>(0,0));
			result.template block<3,1>(3,0)=b.template block<3,1>(12,0)-a.template block<3,1>(12,0);
			return result;
		}

		template<typename T>
		inline static OUTER_T<T> stateTransition(const OUTER_T<T> & state,double time_diff ) {
			assert_inputs(state,time_diff);
			return state;
		}
		inline INNER_T<double> getSTD(double time_diff) {
			INNER_T<double> noise=INNER_T<double>::Ones()*30*time_diff;
			return noise;
		}
	};
}
//zavi::estimator::state_boxes

#endif /* ESTIMATORS_STATEBOXES_HOMOCOORDBOX_HPP_ */
