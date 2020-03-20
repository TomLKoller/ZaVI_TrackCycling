/*
 * XSENSMODEL.hpp
 *
 *  Created on: 04.09.2018
 *      Author: tomlucas
 */

#ifndef ESTIMATORS_MODELS_XSENSMODELWITHBIAS_HPP_
#define ESTIMATORS_MODELS_XSENSMODELWITHBIAS_HPP_
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include "../../Plugins/imu_plugin.hpp"
#include <Eigen_Utils.hpp>
#include "State.hpp"
#include "../StateBoxes/CartesianBox.hpp"
#include "../StateBoxes/OrientationSpeedDerivBox.hpp"
#include "../StateBoxes/AngularRateBiasBox.hpp"
#include "../StateBoxes/AccelerationBiasBox.hpp"
#include "../AlignType.hpp"
namespace zavi
::estimator::model {
	typedef State<zavi::estimator::state_boxes::OrientationSpeedDerivBox,state_boxes::AccelerationBiasBox, state_boxes::AngularRateBiasBox > CALIBRATION_MODEL_TYPE;
	/**
	 * Model for calibration
	 * Assumes that the bycycle only rolls around the ground x-axis
	 */
	class CalibrationModel: public CALIBRATION_MODEL_TYPE {
		//static_assert(std::is_base_of<StateEstimator,state_estimator_type>,"NoOrientationModel template fail state_estimator_type is no base class of StateEstimator");

	public:

		/**
		 * Function to integrate an input measurement
		 * @param state the current state
		 * @param alignment the calibration matrix from imu to body frame
		 * @param prior an arbitrary prior that can be passed to the function
		 * @return the estimated inputs at state
		 */
		struct input_measurement: public base_measurement<6> {
			static inline Eigen::Matrix<double,output_size,output_size> stiffnes;
			template<typename T>
			using MEASURE_TYPE=Eigen::Matrix<T,output_size,1>;
			template<typename T>
			inline MEASURE_TYPE<T> operator()(const OUTER_T<T> &state,const ALIGNMENT_TYPE<T> & alignment, void * prior) {
				MEASURE_TYPE<T> measurement = MEASURE_TYPE < T > ::Zero();
				measurement.template block<3,1>(0, 0) =acceleration_measurement(state,alignment, prior);
				measurement.template block<3,1>(3, 0) = orientation_change_measurement(state,alignment, prior);
				return measurement;
			}
		};
	public:
		CalibrationModel(std::shared_ptr<plugin::SensorPlugin> sensor);
		static constexpr int orientation_start=0;
		static constexpr int orientation_change_start=9;
		static constexpr int orientation_change_deriv_start=12;
		static constexpr int position_start=-1;
		static constexpr int velocity_start=-1;
		static constexpr int acceleration_start=-1;
		static constexpr int velocity_cov_start=-1;
		virtual ~CalibrationModel();

		/**
		 * Function to integrate an acceleration measurement
		 * @param state the current state
		 * @param alignment the calibration matrix from imu to body frame
		 * @param prior an arbitrary prior that can be passed to the function
		 * @return the estimated acceleration at state
		 */
		template<typename T>
		static inline Eigen::Matrix<T, 3, 1> acceleration_measurement(const OUTER_T<T> & state,const ALIGNMENT_TYPE<T> & alignment,
				void * prior) {
			Eigen::Matrix<T,3,3> Q_i_b=alignment.template block<3,3>(0,0);
			Eigen::Matrix<T,3,3> S= eigen_util::makeSkewSymmetric<T>(getOrientationChange(state));
			return Q_i_b*(eigen_util::makeSkewSymmetric<T>(getOrientationChangeDeriv(state))+S*S )*alignment.template block<3,1>(0,3)+state.template block<3,1>(15,0)+state.template block<3,1>(18,0)+Q_i_b*getOrientation(state).transpose()*plugin::IMU_Plugin::gravity;
		}

		/**
		 * Function to integrate an orientation measurement
		 * @param state the current state
		 * @param alignment the calibration matrix from imu to body frame
		 * @param prior an arbitrary prior that can be passed to the function
		 * @return the estimated orientation at state
		 */
		struct orientation_measurement {
			static inline Eigen::Matrix<double,3,3> stiffnes;
			static constexpr inline int output_size=3;
			template<typename T>
			inline Eigen::Matrix<T, 9, 1> operator()(const OUTER_T<T> &state,const ALIGNMENT_TYPE<T>& alignment, void * prior) const {
				return state.template block<9, 1>(orientation_start, 0);
			}
			template<typename T,typename T2>
			static auto boxminus(const Eigen::Matrix<T,9,1> &a,const Eigen::Matrix<T2,9,1> &b) {
				return zavi::eigen_util::boxMinusOrientation<T,T2>(Eigen::Map<const Eigen::Matrix<T,3,3> >(a.data()),Eigen::Map<const Eigen::Matrix<T2,3,3> >(b.data()));
			}
		};

		/**
		 * Function to integrate an acceleration measurement
		 * @param state the current state
		 * @param prior unused
		 * @return the estimated orientation change
		 */
		template<typename T>
		static inline Eigen::Matrix<T, 3, 1> orientation_change_measurement(
				const OUTER_T<T> & state,const Eigen::Matrix<T,4,4> & alignment, void * prior) {
			return alignment.template block<3,3>(0,0)*getOrientationChange(state)+state.template block<3,1>(21,0)+state.template block<3,1>(24,0);
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
		 * get the orientation change in state
		 * @param state the state
		 * @return the orientation change part as 3x3 matrix
		 */
		template<typename T>
		static inline Eigen::Matrix<T, 3, 1> getOrientationChange(const OUTER_T<T> &state) {
			return state.template block<3, 1>(orientation_change_start, 0);
		}
		/**
		 * get the orientation change in state
		 * @param state the state
		 * @return the orientation change part as 3x3 matrix
		 */
		template<typename T>
		static inline Eigen::Matrix<T, 3, 1> getOrientationChangeDeriv(const OUTER_T<T> &state) {
			return state.template block<3, 1>(orientation_change_deriv_start, 0);
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
		 * returns the Position
		 */
		template<typename T>
		static Eigen::Matrix<T, 3, 1> getPosition(const OUTER_T<T> &state) {
			return state.template block<3, 1>(position_start, 0);
		}
		/**
		 * returns the Velocity
		 */
		template<typename T>
		static inline Eigen::Matrix<T, 3, 1> getVelocity(const OUTER_T<T> &state) {
			return state.template block<3, 1>(velocity_start, 0);
		}
		/**
		 * returns the Acceleration
		 */
		template<typename T>
		static inline Eigen::Matrix<T, 3, 1> getAcceleration(const OUTER_T<T> &state) {
			return state.template block<3, 1>(acceleration_start, 0);
		}
		/**
		 * returns the OrientationCov
		 */
		template<typename T>
		static inline Eigen::Matrix<T, 3, 3> getVelocityCov(const Eigen::Matrix<T,inner_size,inner_size> &cov) {
			return cov.template block<3, 3>(velocity_cov_start,3);
		}
	};
}
//zavi::estimator::model

#endif /* ESTIMATORS_MODELS_XSENSMODELWITHBIAS_HPP_ */
