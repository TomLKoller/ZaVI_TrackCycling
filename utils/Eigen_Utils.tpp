/*
 * Eigen_Utils.cpp
 *
 *  Created on: 25.07.2018
 *      Author: tomlucas
 */

#include "Eigen_Utils.hpp"

#include "ZaVI_Utils.hpp"
#include <cassert>
namespace zavi
::eigen_util {
	template<typename T>
	::Eigen::Matrix<T, 3, 3> eulerRodriguez(const T & roll,const T  & pitch,const T  & yaw) {
		assert(abs(roll) <= T(M_PI + 1e-6));
		assert(abs(pitch) <= T(M_PI + 1e-6));
		assert(abs(yaw) <= T(M_PI + 1e-6));
		//T theta = sqrt(pow(roll, 2) + pow(pitch, 2) + pow(yaw, 2) + 1e-8);     //+1e-8 for cers auto diff fix
		T theta=norm(roll,pitch,yaw);
		assert(theta <= T(M_PI + 1e-6));
		T s = theta == T(0.) ? cos(theta) : sin(theta) / theta;
		T c = theta == T(0.) ? 0.5*cos(theta) : (T(1.) - cos(theta)) / pow(theta, 2);
		Eigen::Matrix<T, 3, 3> rotate = Eigen::Matrix<T, 3, 3>::Zero();
		rotate(0, 0) = cos(theta) + c * pow(roll, 2);

		rotate(0, 1) = -s * yaw + c * roll * pitch;
		rotate(1, 0) = s * yaw + c * roll * pitch;

		rotate(0, 2) = s * pitch + c * roll * yaw;
		rotate(2, 0) = -s * pitch + c * roll * yaw;

		rotate(1, 1) = cos(theta) + c * pow(pitch, 2);

		rotate(1, 2) = -s * roll + c * pitch * yaw;
		rotate(2, 1) = s * roll + c * pitch * yaw;

		rotate(2, 2) = cos(theta) + c * pow(yaw, 2);
		assert(abs(rotate.determinant() - T(1)) < T(1e-6));
		return rotate;
	}
	/*	template<typename T>
	 Eigen::Matrix<T,3,1> inverseEulerRodriguez(const ::Eigen::Matrix<T,3,3> & rotation) {
	 assert(abs(rotation.determinant()-T(1))< T(1e-3));
	 T theta=acos(std::min((rotation.trace()-T(1.))/T(2.),T(1.)));     //< with fix for floating point problems (could exceed acos range)
	 zavi::printf(rotation);
	 Eigen::Matrix<T,3,1> angles=Eigen::Matrix<T,3,1>::Zero();
	 angles(0,0)=rotation(2,1)-rotation(1,2);
	 angles(1,0)=rotation(0,2)-rotation(2,0);
	 angles(2,0)=rotation(1,0)-rotation(0,1);
	 angles/=T(2.)*(theta == T(0.) ? T(1.) : sin(theta) / theta);
	 zavi::printf(theta);
	 zavi::printf(T(2.)*(theta == T(0.) ? T(1.) : sin(theta) / theta));
	 angles=wrapRotDeltaVector(angles); // due to numeric instability this is required
	 for(int i=0; i < 3;i++) {
	 assert(abs(angles(i))<=T(M_PI));
	 }
	 return angles;
	 }*/

	/*
	 * new implementation
	 */
	

	template<typename T, int size>
	ceres::Jet<T,size> cutToOne(const ceres::Jet<T,size> & value){
		ceres::Jet<T,size> newValue=value;
		newValue.a=0.99999;
		return newValue;
	}
	
	template<typename T>
	T cutToOne(const T & value){return T(0.99999);}
	
	
	template<typename T>
	Eigen::Matrix<T, 3, 1> inverseEulerRodriguez(const ::Eigen::Matrix<T, 3, 3> & rotation) {

		assert(abs(rotation.determinant() - T(1)) < T(1e-2));
		Eigen::Matrix<T, 3, 1> angles = Eigen::Matrix<T, 3, 1>::Zero();
		angles(0, 0) = rotation(2, 1) - rotation(1, 2);
		angles(1, 0) = rotation(0, 2) - rotation(2, 0);
		angles(2, 0) = rotation(1, 0) - rotation(0, 1);
		T theta;
		if(angles.norm()!=0.) {     // fix 
			theta= atan2(angles.norm(), rotation.trace() - T(1.));
		}
		else {
			T trace=0.5*(rotation.trace() - T(1.));
			if(trace >=T(1.))
				trace=cutToOne(trace);
			theta=acos(trace);
		}
		if (theta !=0. and sin(theta) == T(0.)) {     // 180. degree fix
			angles(0, 0) = sqrt((rotation(0, 0) + 1.) / 2. * pow(theta, 2)+1e-8);

			if (angles(0, 0) == 0.)
			angles(1, 0) = sqrt((rotation(1, 1) + 1.) / 2. * pow(theta, 2)+1e-8);
			else
			angles(1, 0) = rotation(1, 0) / 2. * pow(theta, 2) / angles(1, 0);

			if (angles(0, 0) != 0.)
			angles(2, 0) = rotation(2, 0) / 2. * pow(theta, 2) / angles(0, 0);
			else if (angles(1, 0) != 0.)
			angles(2, 0) = rotation(2, 1) / 2. * pow(theta, 2) / angles(1, 0);
			else
			angles(2, 0) = sqrt((rotation(2, 2) + 1.) / 2. * pow(theta, 2)+1e-8);

		} else {
			angles /= T(2.) * ((theta == T(0.) ? cos(theta) : sin(theta) / theta) );
		}
		angles = wrapRotDeltaVector(angles);     // due to numeric instability this is required
		for (int i = 0; i < 3; i++) {
			assert(abs(angles(i)) <= T(M_PI));
		}
		return angles;
	}

}
//zavi::eigen_util

