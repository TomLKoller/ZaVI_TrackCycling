#ifndef ESTIMATORS_STATEBOXES_LAMBDAPATHBOXNOACCELERATION_HPP_
#define ESTIMATORS_STATEBOXES_LAMBDAPATHBOXNOACCELERATION_HPP_

#include "StateBox.hpp"
#include <eigen3/Eigen/Core>

namespace zavi
::estimator::state_boxes {
	/**
	 * A model for 1D tracks in track frame with velocity and position
	 */
	class LambdaPathBoxNoAcceleration : public StateBox<2,2,1> {
	public:
		INNER_T<double> std;
		LambdaPathBoxNoAcceleration(std::shared_ptr<plugin::SensorPlugin> sensor):StateBox<outer_size,inner_size,input_size>(sensor) {}
		virtual ~LambdaPathBoxNoAcceleration() {};
		template<typename T>
		inline static OUTER_T<T> boxPlus(const OUTER_T<T> &state,const INNER_T<T> &delta) {
			assert_inputs(state,delta);
			return state+delta;
		}
		template<typename T>
		inline static INNER_T<T> boxMinus(const OUTER_T<T> &a,const OUTER_T<T> &b) {
			assert_inputs(a,b);
			return b-a;
		}

		template<typename T>
		inline static OUTER_T<T> stateTransition(const OUTER_T<T> & state,double time_diff ) {
			assert_inputs(state,time_diff);
			OUTER_T<T> result=OUTER_T<T>::Zero();
			result(0,0)=state(0,0)+state(1,0)*time_diff;
			result(1,0)=state(1,0);
			return result;
		}

		INNER_T<double> getSTD(double time_diff) {
			return std*time_diff;
		}
	};
}

#endif //ESTIMATORS_STATEBOXES_LAMBDAPATHBOXNOACCELERATION_HPP_
