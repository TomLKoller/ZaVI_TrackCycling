/*
 * OrientationBox.hpp
 *
 *  Created on: 01.11.2018
 *      Author: tomlucas
 */

#ifndef ESTIMATORS_STATEBOXES_ORIENTATIONBOX_HPP_
#define ESTIMATORS_STATEBOXES_ORIENTATIONBOX_HPP_

#include "StateBox.hpp"
#include <eigen3/Eigen/Core>
#include <Eigen_Utils.hpp>
#include <stdio.h>

namespace zavi
::estimator::state_boxes {
	/**
	 * Orientation modelled as euler rodriguez rotation
	 */
	class OrientationBox : public StateBox<9,3,9> {
		template<typename T>
		using ROT_MATRIX= Eigen::Matrix<T,3,3>;

	public:
		OrientationBox(std::shared_ptr<plugin::SensorPlugin> sensor):StateBox<9,3,9>(sensor) {
		}
		virtual ~OrientationBox() {};

		template<typename T>
		inline static OUTER_T<T> boxPlus(const OUTER_T<T> &state,const INNER_T<T> &delta) {
			assert_inputs(state,delta);
			return Eigen::Map<OUTER_T<T> >(zavi::eigen_util::boxPlusOrientation<T>(Eigen::Map<const ROT_MATRIX<T> >(state.data()),delta).data());
		}
		template<typename T>
		inline static INNER_T<T> boxPlusInnerSpace(const INNER_T<T> &delta1,const INNER_T<T> &delta2) {
			assert_inputs(delta1,delta2);
			INNER_T<T> sum=delta1+delta2;
			sum=zavi::eigen_util::wrapRotDeltaVector(sum);
			return sum;
		}

		template<typename T>
		inline static INNER_T<T> boxMinus(const OUTER_T<T> &a,const OUTER_T<T> &b) {
			assert_inputs(a,b);
			return zavi::eigen_util::boxMinusOrientation<T,T>(Eigen::Map<const ROT_MATRIX<T> >(a.data()),Eigen::Map<const ROT_MATRIX<T> >(b.data()));
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

		static OUTER_T<double> static_normalise(const OUTER_T<double> &state) {
					return Eigen::Map<const OUTER_T<double>>(eigen_util::normaliseRotation(Eigen::Map<const ROT_MATRIX<double> >(state.data()) ).data());
				}
		virtual OUTER_T<double> normalise(const OUTER_T<double> &state) const {
			return static_normalise(state);
		}
	};
}
//zavi::estimator::state_boxes

#endif /* ESTIMATORS_STATEBOXES_ORIENTATIONBOX_HPP_ */
