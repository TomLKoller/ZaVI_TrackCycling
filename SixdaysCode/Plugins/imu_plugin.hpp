/*
 *  Contains the base class for IMU Plugins
 * imu_plugin.hpp
 *
 *  Created on: 24.07.2018
 *      Author: tomlucas
 */

#ifndef PLUGINS_IMU_PLUGIN_HPP_
#define PLUGINS_IMU_PLUGIN_HPP_
#define GRAVITY_CONSTANT 9.81282
#include <ode/ode.h>
#include <vector>
#include <chrono>
#include <thread>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ZaVI_Utils.hpp>

#include "sensor_plugin.hpp"
namespace zavi
::plugin {

	/**
	 * Plugin base class for all plugins simulating or reading an IMU
	 */

	class IMU_Plugin: public SensorPlugin {
	public:
		static inline Eigen::Vector3d gravity=Eigen::Vector3d(0,0,GRAVITY_CONSTANT);     // Gravity vector
		/**
		 * Returns the acceleration in object frame
		 * @return  Vector with x,y,z acceleration
		 */
		virtual inline const Eigen::Vector3d & getAcceleration() {
			return acceleration;
		}
		virtual bool isDataValid(){
			return  data_valid;
		}

		/**
		 * Angular Velocity of Gyroscope
		 *
		 * @return  Vector with angular roll,pitch,yaw
		 */
		virtual inline const Eigen::Vector3d & getAngularVelocity() {
			return angular_velocity;
		}

		/**
		 * Magnetometer values
		 *
		 * @return  Vector with magnetometer
		 */
		virtual inline const Eigen::Vector3d & getMagnetometer() {
			return magnetometer;
		}

		/**
		 * Returns the complete rotation
		 *
		 * @return   orientation as euler angles
		 */
		virtual inline const Eigen::Matrix3d & getOrientation() {
			return orientation;
		}

		/**
		 * Returns the covariance of acceleration
		 * @return Matrix covariance  of x,y,z acceleration
		 */
		virtual inline const Eigen::Matrix3d & getAccelerationCov() {
			return acceleration_cov;
		}
		/**
		 *
		 * @return the covariances of the 6 biases for the acceleration
		 */
		virtual inline const Eigen::Matrix<double, 6, 6> & getAccelerationBiasCov() {
			return acceleration_bias_cov;
		}

		/**
		 *
		 * @return the autocorrelation factors of the 6 biases
		 */
		virtual inline const Eigen::Matrix<double, 6, 1>& getAccelerationBiasCorr() {
			return acceleration_bias_corr;
		}

		/**
		 * Returns the covariance of anuglar rate
		 * @return Matrix covariance  of x,y,z acceleration
		 */
		virtual inline const Eigen::Matrix3d  getAngularRateBiasCov() {
			return angular_rate_bias_cov;
		}
		/**
		 *
		 * @return the correlation time off the angular rate biases
		 */
		virtual inline const Eigen::Vector3d & getAngularRateBiasCorr() {
			return angular_rate_bias_corr;
		}

		/**
		 * Covariance of Angular Velocity of Gyroscope
		 *
		 * @return Matrix covariance  of angular roll,pitch,yaw
		 */
		virtual inline const Eigen::Matrix3d & getAngularVelocityCov() {
			return angular_velocity_cov;
		}

		/**
		 * Covariance of Magnetometer values
		 *
		 * @return Matrix covariance  of magnetometer
		 */
		virtual inline const Eigen::Matrix3d & getMagnetometerCov() {
			return magnetometer_cov;
		}

		/**
		 * Returns the covariance of complete rotation
		 *
		 * @return Matrix covariance  of orientation
		 */
		virtual inline const Eigen::Matrix3d & getOrientationCov() {
			return orientation_cov;
		}

		/**
		 * Virtual Desctructor for later use
		 */
		virtual ~IMU_Plugin() {

		}
		/**
		 * Does not need to construct anything
		 */
		IMU_Plugin():data_valid(true) {

		}

	protected:

		Eigen::Vector3d acceleration, 				//< acceleration in body frame
		angular_velocity,//< angular velocity
		magnetometer;//< magnetometer data

		Eigen::Matrix3d acceleration_cov,//< covariance of acceleration
		angular_velocity_cov,//< covariance of angular velocity
		magnetometer_cov,//< covariance of magnetometer
		orientation_cov,//< covariance of orientation
		orientation;//<orientation data
		Eigen::Matrix<double, 6, 6> acceleration_bias_cov;//covariance of the acceleration biases
		Eigen::Matrix<double, 6, 1> acceleration_bias_corr;// exponential correlation factors of biases
		Eigen::Matrix3d angular_rate_bias_cov; // covariance of the angular velocity biases
		Eigen::Vector3d angular_rate_bias_corr; // correlations of the angular rate biases
		bool data_valid;//!< this flag tells whether the current bit of data is valid or not  (outages for file_imu)
	};

}
//plugin::zavi

#endif /* PLUGINS_IMU_PLUGIN_HPP_ */
