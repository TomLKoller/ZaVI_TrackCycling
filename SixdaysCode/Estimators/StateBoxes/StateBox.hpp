/*
 * StateBox.hpp
 *
 *  Created on: 30.10.2018
 *      Author: tomlucas
 */

#ifndef ESTIMATORS_STATEBOXES_STATEBOX_HPP_
#define ESTIMATORS_STATEBOXES_STATEBOX_HPP_

#include <eigen3/Eigen/Core>
#include "../../Plugins/sensor_plugin.hpp"
namespace zavi
::estimator::state_boxes {
	/**
	 * The basis of every state box which defines the dimensions of the inner and outer state
	 * @template outer_dim the outer (global) dimension of the manifold
	 * @tempalte inner_dim the inner (local) dimension of the manifold
	 * @template input_dim the dimension of the input (input_dim values at the end of the state are not considered for transition costs in batch estimator)
	 */
	template<int outer_dim,int inner_dim,int input_dim>
	class StateBox {
	public:
		StateBox(std::shared_ptr<plugin::SensorPlugin> sensor) {}
		template<typename T>
		using OUTER_T=typename Eigen::Matrix<T,outer_dim,1>;
		template<typename T>
		using INNER_T=typename Eigen::Matrix<T,inner_dim,1>;
		static const int inner_size=inner_dim;
		static const int outer_size=outer_dim;
		static const int input_size=input_dim;
		virtual ~StateBox() {};
		//State boxes need to implement those 3 functions but templates cant be virtual
		/*template<typename T>
		 OUTER_T<T> boxPlus(const OUTER_T<T> &state,const INNER_T<T> &delta)=0;
		 template<typename T>
		 virtual INNER_T<T> boxMinus(const OUTER_T<T> &a,const OUTER_T<T> &b)=0;
		 template<typename T>
		 virtual OUTER_T<T> stateTransition(const OUTER_T<T> & state,double time_diff )=0; */
		/**
		 * Boxplus in inner spcae
		 * @param delta1
		 * @param delta2
		 * @return delta1 boxplus delta2
		 */
		template<typename T>
		static inline INNER_T<T> boxPlusInnerSpace(const INNER_T<T> &delta1,const INNER_T<T> &delta2) {
			return delta1+delta2;
		}
		/**
		 * normalises a given state. Essential for orientation matrices
		 * @param state
		 * @return a numerically normalised version of the state
		 */
		virtual OUTER_T<double> normalise(const OUTER_T<double> &state) const {return state;}

	};

}

#endif /* ESTIMATORS_STATEBOXES_STATEBOX_HPP_ */
