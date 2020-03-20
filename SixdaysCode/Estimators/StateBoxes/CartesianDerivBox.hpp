#ifndef ESTIMATORS_STATEBOXES_CARTESIANDERIVBOX_HPP_
#define ESTIMATORS_STATEBOXES_CARTESIANDERIVBOX_HPP_

#include "StateBox.hpp"
#include <eigen3/Eigen/Core>

namespace zavi
::estimator::state_boxes {
	/**
	 * Models a 3D Kinematik in world coordinates consiting of acceleration derivative, acceleration, velocity and position
	 */

	class CartesianDerivBox : public StateBox<12,12,3> {
		template<typename T>
		using CART_MATRIX= Eigen::Matrix<T,3,4>;
	public:
		INNER_T<double> std;
		CartesianDerivBox(std::shared_ptr<plugin::SensorPlugin> sensor):StateBox<outer_size,inner_size,input_size>(sensor) {}
		virtual ~CartesianDerivBox() {};
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
			static Eigen::Matrix<double,4,4> transition=transition.Identity();
			transition(1,0)=transition(2,1)=time_diff;
			//transition(2,1)=time_diff;
			transition(2,0)=pow(time_diff,2)*0.5;
			transition(3,2)=time_diff;
			static bool first=true;
			CART_MATRIX<T> result = Eigen::Map<const CART_MATRIX<T>>(state.data()) *transition;
			return Eigen::Map<OUTER_T<T> >(result.data());
		}
		/*static INNER_T<double> getSTD(double time_diff) {
		 static INNER_T<double> noise=INNER_T<double>::Zero();
		 noise(0)=noise(1)=noise(2)=time_diff*0.1;
		 noise (3)=noise (4)=noise (5)=time_diff*0.1;
		 noise(6)=noise(7)=noise(8)=time_diff*100;
		 return noise;
		 }*/

		/*static INNER_T<double> getSTD(double time_diff) {
		 static INNER_T<double> noise=INNER_T<double>::Zero();
		 noise(0)=noise(1)=noise(2)=time_diff*5;
		 noise (3)=noise (4)=noise (5)=time_diff;
		 noise(6)=noise(7)=noise(8)=time_diff*100;
		 return noise;
		 }*/
		INNER_T<double> getSTD(double time_diff) {
			return std*time_diff;
		}
	};
}

#endif //ESTIMATORS_STATEBOXES_CARTESIANDERIVBOX_HPP_
