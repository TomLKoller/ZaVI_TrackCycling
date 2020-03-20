/*
 * BikeInfo.hpp
 *
 *  Created on: 18.02.2019
 *      Author: tomlucas
 */

#ifndef SIXDAYS_BIKEINFO_HPP_
#define SIXDAYS_BIKEINFO_HPP_


#include <Eigen/Core>
namespace zavi::sixdays{
	struct BikeInfo{
		Eigen::Vector3d imu_to_front, imu_to_back;

		Eigen::Vector3d imuToFront(){
			return imu_to_front;
		}
		Eigen::Vector3d imuToBack(){
			return imu_to_back;
		}
	};
}




#endif /* SIXDAYS_BIKEINFO_HPP_ */
