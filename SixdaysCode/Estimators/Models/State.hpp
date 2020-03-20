/*
 * State.hpp
 *
 *  Created on: 30.10.2018
 *      Author: tomlucas
 */

#ifndef ESTIMATORS_MODELS_STATE_HPP_
#define ESTIMATORS_MODELS_STATE_HPP_
#include "../StateBoxes/StateBox.hpp"
#include <tuple>
#include "../../Plugins/sensor_plugin.hpp"
#include <boost/hana.hpp>
#include <boost/hana/ext/std/tuple.hpp>
#define INNER_TYPE 0
#define OUTER_TYPE 1

/**
 * The State Type Reader reads the sizes of the state boxes and tells the state how big the matrices have to be
 */
template<int inner, int outer, int input, typename ... ARGS>
class StateTypeReader {
public:
	static const int inner_, outer_;
};

template<int inner, int outer, int input, typename first, typename ... ARGS>
class StateTypeReader<inner, outer, input, first, ARGS...> : public StateTypeReader<inner + first::inner_size,
		outer + first::outer_size, input + first::input_size, ARGS ...> {

};

template<int inner, int outer, int input>
class StateTypeReader<inner, outer, input> {
public:
	static const int inner_size = inner;
	static const int outer_size = outer;
	static const int input_size = input;
	template<typename T>
	using OUTER_T=Eigen::Matrix<T,outer,1>;
	template<typename T>
	using INNER_T=Eigen::Matrix<T,inner,1>;
	template<typename T>
	using INPUT_T=Eigen::Matrix<T,input,1>;
};

namespace zavi
::estimator::model {
	/**
	 * default implementation for boxminus
	 * @param a
	 * @param b
	 * @return b boxminus a
	 */
	template< int measure_dim>
	struct base_measurement {
		static constexpr inline int output_size=measure_dim;
		template<typename T,typename T2>
		static auto boxminus(const Eigen::Matrix<T,measure_dim,1> & a, const Eigen::Matrix<T2,measure_dim,1> & b) {
			return b-a;
		}
	};

	/**
	 * The state is constructed from so called state boxes
	 * The state boxes implement a dynamic model, have inputs values, a boxplus and a boxminus operation
	 * This class just collects the state boxes from the STATE_BOXES tuple and passes eigen matrices to the functions of the boxes
	 * For Estimators, it provides a simple interface with  an eigen vector containing all states and functions to edit it
	 */
	template<typename ... STATE_BOXES>
	class State {
	public:
		// Reads the sizes and creates all types
		typedef StateTypeReader<0,0,0,STATE_BOXES ...> STATE_TYPE;

		typedef typename STATE_TYPE::template OUTER_T<double> OUTER_DOUBLE;
		static const int inner_size=STATE_TYPE::inner_size;
		static const int outer_size=STATE_TYPE::outer_size;
		static const int input_size=STATE_TYPE::input_size;
		static const int cost_size=inner_size-input_size;
		//static const int cost_size=outer_size-input_size;
		template<typename T>
		using OUTER_T=typename STATE_TYPE::template OUTER_T<T>;
		template<typename T>
		using INNER_T=typename STATE_TYPE::template INNER_T<T>;
		template<typename T>
		using INPUT_T=typename STATE_TYPE::template INPUT_T<T>;
		template<typename T>
		using COST_T=Eigen::Matrix<T,cost_size,1>;;
		typedef std::tuple<STATE_BOXES ...> BOXES_TUPLE;
		BOXES_TUPLE boxes;
		State(std::shared_ptr<plugin::SensorPlugin> sensor):boxes(initStates<0,STATE_BOXES ...>(sensor)) {

		}
		virtual ~State() {}
		/**
		 *
		 * @return The outer size of the state
		 */
		virtual int GlobalSize() const {
			return outer_size;
		}

		/**
		 *
		 * @return The inner size of the state
		 */
		virtual int LocalSize() const {
			return inner_size;
		}

		/**
		 * passes box_plus to ceres
		 * @param x
		 * @param delta
		 * @param x_plus_delta
		 * @return
		 */
		template<typename T>
		bool Plus(const T * x, const T* delta, T* x_plus_delta) const {
			(Eigen::Map<OUTER_T<T> >(x_plus_delta))=boxPlus<T>(Eigen::Map<const OUTER_T<T> >(x),Eigen::Map<const INNER_T<T> >(delta));
			return true;
		}

		/**
		 * Passes box_plus to ceres
		 * @param x
		 * @param delta
		 * @param x_plus_delta
		 * @return
		 */
		template<typename T>
		bool operator()(const T* x, const T* delta, T* x_plus_delta) const {
			return Plus<T>(x,delta,x_plus_delta);

		}

		/**
		 * Pass the sensor to all state boxes
		 * @param sensor the sensor plugin (IMU PLugin)
		 * @return a tuple with all boxes
		 */
		template<int current_depth,typename T,typename ... rest>
		BOXES_TUPLE initStates(std::shared_ptr<plugin::SensorPlugin> sensor) {
			std::get<current_depth>(boxes)=T(sensor);
			return initStates<current_depth+1,rest ...>(sensor);
		}
		/**
		 * Just for the last recursion step, does nothing
		 * @param sensor
		 * @return
		 */
		template<int current_depth>
		BOXES_TUPLE initStates(std::shared_ptr<plugin::SensorPlugin> sensor) {
			return boxes;
		}

		/**
		 * Splits the state  and inner vector and passes them to all state boxes
		 * @param state
		 * @param delta
		 * @return state boxplis delta
		 */
		template<typename T>
		OUTER_T<T> boxPlus(const OUTER_T<T> & state,const INNER_T<T> & delta) const {
			OUTER_T<T> result=OUTER_T<T>::Zero();
			int outer_index=0,inner_index=0;
			boost::hana::for_each(boxes,[&result,&state,&delta,&outer_index, &inner_index](auto const & foo) {
						result.template block<foo.outer_size,1>(outer_index,0)=foo.template boxPlus<T>(state.template block<foo.outer_size,1>(outer_index,0),delta.template block<foo.inner_size,1>(inner_index,0));
						outer_index+=foo.outer_size;
						inner_index+=foo.inner_size;
					});
			return result;
		}
		/**
		 * as boxPlus, but for the inner state
		 * It is needed if the inner space itself is a manifold  (orientations)
		 * @param delta1
		 * @param delta2
		 * @return
		 */
		template<typename T>
		INNER_T<T> boxPlusInnerSpace(const INNER_T<T> & delta1,const INNER_T<T> & delta2) const {
			INNER_T<T> result=INNER_T<T>::Zero();
			int inner_index=0;
			boost::hana::for_each(boxes,[&result,&delta1,&delta2, &inner_index](auto const & foo) {
						result.template block<foo.inner_size,1>(inner_index,0)=foo.template boxPlusInnerSpace<T>(delta1.template block<foo.inner_size,1>(inner_index,0),delta2.template block<foo.inner_size,1>(inner_index,0));
						inner_index+=foo.inner_size;
					});
			return result;
		}
		/**
		 * As boxPlus but with boxMinus
		 * @param a
		 * @param b
		 * @return
		 */
		template<typename T>
		INNER_T<T> boxMinus(const OUTER_T<T> & a,const OUTER_T<T> & b) {
			INNER_T<T> result=INNER_T<T>::Zero();
			int outer_index=0,inner_index=0;
			boost::hana::for_each(boxes,[&result,&a,&b,&outer_index, &inner_index](auto & foo) {
						result.template block<foo.inner_size,1>(inner_index,0)=foo.template boxMinus<T>(a.template block<foo.outer_size,1>(outer_index,0),b.template block<foo.outer_size,1>(outer_index,0));
						outer_index+=foo.outer_size;
						inner_index+=foo.inner_size;
					});
			return result;
		}

		/**
		 * Calls the state transition function on all boxes
		 * @param state the current state
		 * @param time_diff the passed time till last call
		 * @return the new state
		 */
		template<typename T>
		OUTER_T<T> stateTransitionFunction(const OUTER_T<T> & state,const double time_diff) {
			OUTER_T<T> result=OUTER_T<T>::Zero();
			int outer_index=0;
			boost::hana::for_each(boxes,[&result,&state,time_diff,&outer_index](auto & foo) {
						result.template block<foo.outer_size,1>(outer_index,0)=foo.template stateTransition<T>(state.template block<foo.outer_size,1>(outer_index,0),time_diff);
						outer_index+=foo.outer_size;
					});
			return result;
		}
		/*template<typename T>
		 static INPUT_T<T> input_measurement(const OUTER_T<T> & state,void * prior) {
		 BOXES_TUPLE *box=static_cast<BOXES_TUPLE*>(*prior);
		 INPUT_T<T> result;
		 int outer_index=0,input_index=0;
		 boost::hana::for_each(*box,[&result,&state,&outer_index,&input_index](auto & foo) {
		 outer_index+=foo.outer_size;
		 result.template block<foo.input_size,1>(input_index,0)=state.template block<foo.input_size,1>(outer_index-foo.input_size,0);
		 input_index+=foo.input_size;
		 });
		 return result;
		 }*/

		/**
		 * Calculates the transition cost between to states scaled by stiffness
		 * @param state the current state
		 * @param time_diff the üassed time between 2 states
		 * @param after the next state
		 * @return stiffness*(after boxminus transition(state))
		 */
		template<typename T>
		COST_T<T> transitionCost(const OUTER_T<T> & state,const double time_diff, const OUTER_T<T> & after) {
			Eigen::Matrix<double,cost_size,cost_size> stiffnes=(getTransitionSTD(time_diff)+COST_T<double>::Ones()*1e-6).cwiseInverse().asDiagonal();
			//zavi::plot::liveHeatMap<0>(stiffnes," stiffnes of transition");
			COST_T<T> result=COST_T<T>::Zero();
			int outer_index=0,cost_index=0;
			boost::hana::for_each(boxes,[&result,&state,time_diff,&after,&outer_index,&cost_index](auto & foo) {
						result.template block<foo.inner_size-foo.input_size,1>(cost_index,0)=foo.template boxMinus<T>(foo.template stateTransition<T>(state.template block<foo.outer_size,1>(outer_index,0),time_diff),after.template block<foo.outer_size,1>(outer_index,0)).template block<foo.inner_size-foo.input_size,1>(0,0);
						//result.template block<foo.outer_size-foo.input_size,1>(cost_index,0)=(after.template block<foo.outer_size,1>(outer_index,0)-foo.template stateTransition<T>(state.template block<foo.outer_size,1>(outer_index,0),time_diff)).template block<foo.outer_size-foo.input_size,1>(0,0);
						cost_index+=foo.inner_size-foo.input_size;
						//cost_index+=foo.inner_size-foo.input_size;
						outer_index+=foo.outer_size;
					});
			assert_inputs(stiffnes,result);
			return stiffnes*result;
			//return result;
		}
		/**
		 * Get the transition standard deviation (without inputs)
		 * @param time_diff of the transition
		 * @return a cost_T vector containing the stds
		 */
		COST_T<double> getTransitionSTD(const double time_diff) {
			COST_T<double> result=COST_T<double>::Zero();
			int outer_index=0,cost_index=0;
			boost::hana::for_each(boxes,[&result,time_diff,&outer_index,&cost_index](auto & foo) {
						result.template block<foo.inner_size-foo.input_size,1>(cost_index,0)=foo.getSTD(time_diff).template block<foo.inner_size-foo.input_size,1>(0,0);
						cost_index+=foo.inner_size-foo.input_size;
						outer_index+=foo.outer_size;
					});
			return result;
		}
		/**
		 * Get the standard deviation  (with inputs)
		 * @param time_diff of the transition
		 * @return a cost_T vector containing the stds
		 */
		INNER_T<double> getStateSTD(const double time_diff) {
			INNER_T<double> result=INNER_T<double>::Zero();
			int inner_index=0;
			boost::hana::for_each(boxes,[&result,time_diff,&inner_index](auto & foo) {
						result.template block<foo.inner_size,1>(inner_index,0)=(foo.getSTD(time_diff));
						inner_index+=foo.inner_size;
					});
			return result;
		}

		/**
		 * Call normalise function of all state boxes
		 * generally needed for orientation boxes to prevent numeric failures
		 * @param state the current state
		 * @return a normalised version of the state
		 */
		OUTER_T<double> normalise(const OUTER_T<double> & state) const {
			OUTER_T<double> result=result.Zero();
			int outer_index=0;
			boost::hana::for_each(boxes,[&result,&state,&outer_index](auto const & foo) {
						result.template block<foo.outer_size,1>(outer_index,0)=foo.normalise(state.template block<foo.outer_size,1>(outer_index,0));
						outer_index+=foo.outer_size;
					});
			return result;
		}

	};
}

#endif /* ESTIMATORS_MODELS_STATE_HPP_ */
