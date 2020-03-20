/*
 * XSENSMODEL.hpp
 *
 *  Created on: 04.09.2018
 *      Author: tomlucas
 */

#ifndef ESTIMATORS_MODEL_LAMBDAPATHWITHVELOCITYMODEL_HPP_
#define ESTIMATORS_MODEL_LAMBDAPATHWITHVELOCITYMODEL_HPP_
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include "../../Plugins/imu_plugin.hpp"
#include <Eigen_Utils.hpp>
#include "State.hpp"
#include "../StateBoxes/LambdaPathBoxNoAcceleration.hpp"
#include "../StateBoxes/OrientationSpeedBox.hpp"
#include "../StateBoxes/CartesianBox.hpp"
namespace zavi
::estimator::model {
	typedef State<state_boxes::CartesianBox,state_boxes::LambdaPathBoxNoAcceleration, state_boxes::OrientationSpeedBox> LAMBDA_PATH_WITH_VELOCITY_MODEL_TYPE;
	/**
	 * Model for 1D movement with forward velocity prior
	 */
	class LambdaPathWithVelocityModel: public LAMBDA_PATH_WITH_VELOCITY_MODEL_TYPE {
		//static_assert(std::is_base_of<StateEstimator,state_estimator_type>,"NoOrientationModel template fail state_estimator_type is no base class of StateEstimator");

	public:
		static constexpr int orientation_start=11;
		static constexpr int orientation_speed_start=20;
		static constexpr int velocity_start=3;
		static constexpr int lambda_start=9;
		static constexpr int lambda_velocity_start=10;
		static constexpr int acceleration_start=6;
		LambdaPathWithVelocityModel(std::shared_ptr<plugin::SensorPlugin> sensor): LAMBDA_PATH_WITH_VELOCITY_MODEL_TYPE(sensor) {
			plugin::IMU_Plugin * imu=static_cast<plugin::IMU_Plugin *>(sensor.get());
			Eigen::Matrix<double,input_size,input_size> stiffnes;
			stiffnes=stiffnes.Zero();
			stiffnes.block<3,3>(0,0)=imu->getAccelerationCov();
			input_measurement::stiffnes=stiffnes.diagonal().cwiseInverse().cwiseSqrt().asDiagonal();
		}

		virtual ~LambdaPathWithVelocityModel() {

		}
		/**
		 * Function to integrate an input measurement
		 * @param state the current state
		 * @param alignment the calibration matrix from imu to body frame
		 * @param prior an arbitrary prior that can be passed to the function
		 * @return the estimated input at state
		 */
		struct input_measurement: public base_measurement<input_size> {
			static inline Eigen::Matrix<double,input_size,input_size> stiffnes;
			template<typename T>
			inline INPUT_T<T> operator()(const OUTER_T<T> &state,const Eigen::Matrix<T,4,4> & alignment, void * prior) {
				INPUT_T<T> measurement = INPUT_T < T > ::Zero();
				measurement.template block<3,1>(0, 0) =acceleration_measurement(state,alignment, prior);
				return measurement;
			}
		};

		/**
		 * Measurement to set the lambda velocity to the norm of the cartesian velocity
		 * @param state the current state
		 * @param alignment the calibration matrix from imu to body frame
		 * @param prior an arbitrary prior that can be passed to the function
		 * @return The difference between the cartesian velocity norm and the lambda velocity
		 */
		template<typename T>
		static inline Eigen::Matrix<T, 1, 1> dot_lambda_measurement(const OUTER_T<T> & state,const Eigen::Matrix<T,4,4> & alignment,
				void * prior) {
			Eigen::Matrix<T, 1, 1> measurement = Eigen::Matrix<T, 1, 1> ::Zero();
			measurement(0, 0) =getVelocity(state).norm()-getLambdaVelocity(state);
			return measurement;
		}

		/**
		 * Function to integrate an acceleration measurement
		 * @param state the current state
		 * @param alignment the calibration matrix from imu to body frame
		 * @param prior an arbitrary prior that can be passed to the function
		 * @return the estimated acceleration at state
		 */
		template<typename T>
		static inline Eigen::Matrix<T, 3, 1> acceleration_measurement(const OUTER_T<T> & state,const Eigen::Matrix<T,4,4> & alignment,
				void * prior) {
			Eigen::Matrix<T,3,3> Q_i_b=alignment.template block<3,3>(0,0);
			Eigen::Matrix<T,3,3> S= eigen_util::makeSkewSymmetric<T>(getOrientationChange(state));
			return Q_i_b*getOrientation(state).transpose() * ((state.template block<3,1>(acceleration_start, 0) + zavi::plugin::IMU_Plugin::gravity))+Q_i_b*(S*S )*alignment.template block<3,1>(0,3);

		}
		/**
		 * get the orientation in state
		 * @param state the state
		 * @return the orientation part as 3x3 matrix
		 */
		template<typename T>
		static inline Eigen::Matrix<T, 3, 3> getOrientation(const OUTER_T<T> &state) {
			return Eigen::Map<const Eigen::Matrix<T, 3, 3> >(state.template block<9,1>(orientation_start, 0).data());
		}

		/**
		 * Function to integrate an acceleration measurement
		 * @param state the current state
		 * @param prior unused
		 * @return the estimated orientation change
		 */
		template<typename T>
		static inline Eigen::Matrix<T, 3, 1> orientation_change_measurement(
				const OUTER_T<T> & state,const Eigen::Matrix<T,4,4> & alignment, void * prior) {
			return alignment.template block<3,3>(0,0)*getOrientationChange(state);
		}

		/**
		 * get the orientation change in state
		 * @param state the state
		 * @return the orientation change part as 3x3 matrix
		 */
		template<typename T>
		static inline Eigen::Matrix<T, 3, 1> getOrientationChange(const OUTER_T<T> &state) {
			return state.template block<3, 1>(orientation_speed_start, 0);
		}
		/**
		 * Turn rotation matrix into 9x1 block
		 * @param rotation the rotation matrix
		 * @return a 9x1 block containing the rotation matrix
		 */
		template<typename T>
		static inline Eigen::Matrix<T, 9, 1> toStateOrientation(const Eigen::Matrix<T, 3, 3> &rotation) {
			return Eigen::Map<Eigen::Matrix<T, 9, 1> >(rotation.data());
		}

		/**
		 * returns the Velocity
		 */
		template<typename T>
		static inline Eigen::Matrix<T, 3, 1> getVelocity(const OUTER_T<T> &state) {
			return state.template block<3, 1>(velocity_start, 0);
		}
		/**
		 * returns the Lambda Velocity
		 */
		template<typename T>
		static inline T getLambdaVelocity(const OUTER_T<T> &state) {
			return state(lambda_velocity_start);
		}

	};
}
//zavi::estimator::model

#endif /* ESTIMATORS_MODEL_LAMBDAPATHWITHVELOCITYMODEL_HPP_ */
