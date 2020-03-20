/*
 * StateReaderCallback.hpp
 *
 *  Created on: 13.02.2019
 *      Author: tomlucas
 */

#ifndef LAMBDA_PATH_STATE_READER_CALLBACK_HPP
#define LAMBDA_PATH_STATE_READER_CALLBACK_HPP
#include <Callbacker.hpp>
#include <vector>
#include "../../Plugins/sensor_plugin.hpp"
namespace zavi
::estimator::callbacks {

	/**
	 * state size is the state size of the target estimator
	 * IMportant: only works for a batch estimator which has a CartesianBox followed by an OrientationSpeed Box at first
	 */
	template<int state_size, typename lambda_path_estimator>
	class LambdaPathSmoothedReaderCallback: public Estimator,public Callback {

		/**
		 * reads the values of a lambda path callback for position and then calculates a fitting orientation and velocity
		 */
		struct InternalCallback: public Callback {
			volatile unsigned int counter; //!< Counter for replay, the current state which is shown
			volatile unsigned int size; //!< the size of the states array
			std::vector<Eigen::Matrix<double, state_size, 1> > states;
			InternalCallback():counter(0),size(0) {

			}
			/**
			 * Retrieves the lambda values of the lambda ukf and calculates a state for the estimator
			 * @param callbacker
			 */
			virtual void operator()(void * callbacker) {
				lambda_path_estimator * estimator = static_cast<lambda_path_estimator *>(callbacker);

				auto smoothed_states = estimator->getSmoothedStates();
				unsigned int internal_size=0;
				for (auto state : smoothed_states) {
					Eigen::Matrix<double, state_size, 1> calculated_state = calculated_state.Zero();

					calculated_state.template block<3, 1>(0, 0) = estimator->getFuncCaller()(state(estimator->getBoxModel().lambda_start, 0));
					calculated_state.template block<3, 1>(3, 0) = abs(state(estimator->getBoxModel().lambda_velocity_start,0))*zavi::eigen_util::diff1(state(estimator->getBoxModel().lambda_start,0),estimator->getFuncCaller(), 1e-5);
					//calculated_state.template block<3, 1>(6, 0) = state(2,0)*zavi::eigen_util::diff2(state(estimator->getBoxModel().lambda_start,0),estimator->getFuncCaller(), 1e-5);
					calculated_state.template block<9, 1>(9, 0) = Eigen::Matrix<double,9,1>(zavi::eigen_util::orientationFromFunctor(state(estimator->getBoxModel().lambda_start,0),estimator->getFuncCaller(),1e-5).data());
					calculated_state.template block<3, 1>(18, 0) = state.template block<3, 1>(estimator->getBoxModel().orientation_speed_start, 0);

					states.push_back(calculated_state);
					internal_size++;
				}
				size=internal_size;
			}
			virtual void operator()(plugin::SensorPlugin *, void * estimator, double time) {
				operator()(estimator);
			}
		};

	public:
		//Type Erasure for template parameters functionality in INternalCallback
		std::shared_ptr<InternalCallback> callback;
		LambdaPathSmoothedReaderCallback():callback(new InternalCallback()) {

		}
		virtual ~LambdaPathSmoothedReaderCallback() {
		}
		virtual void operator()(plugin::SensorPlugin *, void * estimator, double time) {
			callback->operator()(estimator);
		}

		virtual void operator()(void * callbacker) {
			callback->operator()(callbacker);
		}

		void run(double freq, bool (* running)()) {
			Timer loop(freq);
			while(running()) {
				if(callback->size >0) {
					if(callback->counter< callback->size-1) {
						callback->counter++;
						}
					else {
						LOG(INFO) << "Finished replaying  ukf_saver";
						return;
					}
				}
				loop.wait();
			}

		}

		virtual Eigen::Vector3d getEstimatedPosition() {
			if(callback->counter < callback->states.size()) {
				return callback->states[callback->counter].template block<3,1>(0,0);
			}
			else
			return Eigen::Vector3d::Zero();
		}

		virtual Eigen::Quaterniond getEstimatedOrientation() {
			if(callback->counter < callback->states.size())
			return Eigen::Quaterniond(Eigen::Map<Eigen::Matrix3d>((callback->states[callback->counter].template block<9,1>(9,0) ).data()));
			else
			return Eigen::Quaterniond(0,0,0,1);

		}
		virtual Eigen::Matrix3d getEstimatedPositionError() {
			throw "Not Implemented GetEstimatedPositionError";
		}
	};
}

#endif /* LAMBDA_PATH_STATE_READER_CALLBACK_HPP */
