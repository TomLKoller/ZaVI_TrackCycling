/*
 * OrientationBox.hpp
 *
 *  Created on: 01.11.2018
 *      Author: tomlucas
 */

#ifndef ESTIMATORS_STATEBOXES_ORIENTATIONSPEEDDERIVBOX_HPP_
#define ESTIMATORS_STATEBOXES_ORIENTATIONSPEEDDERIVBOX_HPP_

#include "StateBox.hpp"
#include <eigen3/Eigen/Core>
#include <Eigen_Utils.hpp>
#include "OrientationSpeedBox.hpp"
#include <cassert>
namespace zavi
::estimator::state_boxes {
	/**
	 * Box to contain orientation and angular velocity and angular acceleration
	 */
	class OrientationSpeedDerivBox : public StateBox<15,9,3> {
		template<typename T>
		using ROT_MATRIX= Eigen::Matrix<T,3,3>;
	public:
		INNER_T<double> std;
		OrientationSpeedDerivBox(std::shared_ptr<plugin::SensorPlugin> sensor):StateBox<outer_size,inner_size,input_size>(sensor) {
		}
		virtual ~OrientationSpeedDerivBox() {};

		template<typename T>
		inline static OUTER_T<T> boxPlus(const OUTER_T<T> &state,const INNER_T<T> &delta) {
			assert_inputs(state,delta);
			OUTER_T<T> result;
			result <<OrientationBox::boxPlus<T>(state.template block<9,1>(0,0),delta.template block<3,1>(0,0)),(state.template block<6,1>(9,0)+delta.template block<6,1>(3,0));

			return result;
		}
		template<typename T>
		inline static INNER_T<T> boxPlusInnerSpace(const INNER_T<T> &delta1,const INNER_T<T> &delta2) {
			assert_inputs(delta1,delta2);
			INNER_T<T> result=INNER_T<T>::Zero();
			result.template block<3,1>(0,0)=OrientationBox::boxPlusInnerSpace<T>(delta1.template block<3,1>(0,0),delta2.template block<3,1>(0,0));
			result.template block<6,1>(3,0)=delta1.template block<6,1>(3,0)+delta2.template block<6,1>(3,0);
			return result;
		}

		template<typename T>
		inline static INNER_T<T> boxMinus(const OUTER_T<T> &a,const OUTER_T<T> &b) {
			assert_inputs(a,b);
			INNER_T<T>result;
			result <<OrientationBox::boxMinus<T>(a.template block<9,1>(0,0),b.template block<9,1>(0,0)),b.template block<6,1>(9,0)-a.template block<6,1>(9,0);
			return result;
		}
		template<typename T>
		inline static OUTER_T<T> stateTransition(const OUTER_T<T> & state,double time_diff ) {
			assert_inputs(state,time_diff);
			OUTER_T<T> result;
			result << OrientationBox::boxPlus<T>(state.template block<9,1>(0,0),state.template block<3,1>(9,0)*time_diff),state.template block<3,1>(12,0)*time_diff+state.template block<3,1>(9,0),state.template block<3,1>(12,0);
			return result;
		}
		inline INNER_T<double> getSTD(double time_diff) {
			assert_inputs(std);
			return std*time_diff;
		}

		virtual OUTER_T<double> normalise(const OUTER_T<double> &state) const {
			assert_inputs(state);
			OUTER_T<double> result;
			result <<OrientationBox::static_normalise(state.block<9,1>(0,0)),state.block<6,1>(9,0);
			return result;
		}
	};
}

#endif /* ESTIMATORS_STATEBOXES_ORIENTATIONSPEEDDERIVBOX_HPP_ */
