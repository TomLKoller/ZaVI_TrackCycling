/*
 * @file File for Eigen helper functions
 * Eigen_Utils.hpp
 *
 *  Created on: 25.07.2018
 *      Author: tomlucas
 */

#ifndef EIGEN_UTILS_HPP_
#define EIGEN_UTILS_HPP_

#include <Eigen/Geometry>
#include <math.h>
#include "ZaVI_Utils.hpp"
#include <ceres/ceres.h>
namespace zavi
::eigen_util {
	/**
	 * Rotates a vector with the given quaternion
	 * @param vector The vector to rotate
	 * @param quat The quaternion
	 * @return the rotatet vector
	 */
	inline ::Eigen::Vector3d rotateVector(const ::Eigen::Vector3d &vector, const ::Eigen::Quaterniond &quat) {
		::Eigen::Quaterniond vec(0, 0, 0, 0);
		vec.w() = 0;
		vec.vec() = vector;
		return (quat * vec * quat.conjugate()).vec();
	}
	/**
	 * Euler rodriguez formula
	 *
	 * Calculates a rotation matrix as if roll pitch and yaw happend simultaenously
	 * taken from C. Hertzberg 2013
	 * @param roll the roll angle
	 * @param pitch   the pitch angle
	 * @param yaw  the yaw angle
	 * @return a 3x3 rotation matrix
	 */
	template<typename T>
	::Eigen::Matrix<T, 3, 3> eulerRodriguez(const T & roll,const T & pitch,const T & yaw);

	/**
	 * Euler rodriguez formula
	 *
	 * Calculates a rotation matrix as if roll pitch and yaw happend simultaenously
	 * taken from C. Hertzberg 2013
	 * @param vec roll pitch and yaw as vector
	 * @return a 3x3 rotation matrix
	 */
	template<typename T>
	inline ::Eigen::Matrix<T, 3, 3> eulerRodriguez(const Eigen::Matrix<T, 3, 1> &vec) {
		return eulerRodriguez<T>(vec(0, 0), vec(1, 0), vec(2, 0));
	}

	/**
	 * Wraps an angle to -PI + PI
	 * @param angle the angle
	 * @return the wrapped angle
	 */
	template<typename T>
	T wrapAngle(const T & angle) {
		T temp=angle+M_PI;
		temp = fmod(temp,2*M_PI);
		if (temp < T(0.))
		temp += 2*M_PI;
		return temp - M_PI;
	}

	/**
	 * Wraps an angle to -PI + PI
	 * @param angle the angle
	 * @return the wrapped angle
	 */
	template<typename T,int size>
	ceres::Jet<T,size> wrapAngle(const ceres::Jet<T,size> & angle) {
		ceres::Jet<T,size> temp=angle;
		temp.a=wrapAngle(temp.a);
		return temp;
	}
	/**
	 * wraps the angles to -M_PI to M_PI
	 * @param matrix a eigen matrix or expression
	 * @return
	 */
	template<typename Derived>
	inline ::Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, 1> wrapAngles(
			const ::Eigen::MatrixBase<Derived> & matrix) {
		::Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, 1> temp = matrix;
		for (int i = 0; i < matrix.RowsAtCompileTime; i++) {
			temp(i) = wrapAngle(temp(i));
		}
		return temp;
	}
	/**
	 * retrieve the euler angles from an euler rodriguez rotation matrix
	 * @param rotation the rotation matrix
	 * @return euler angles as 3,1 matrix
	 */
	template<typename T>
	Eigen::Matrix<T, 3, 1> inverseEulerRodriguez(const ::Eigen::Matrix<T, 3, 3> & rotation);

	/**
	 * Generates an quaternion from euler angle
	 *
	 *taken from https://stackoverflow.com/questions/31589901/euler-to-quaternion-quaternion-to-euler-using-eigen
	 *
	 * @param roll  roll in radians
	 * @param pitch pitch in radians
	 * @param yaw  yaw in radians
	 * @return
	 */
	inline ::Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw) {
		Eigen::Quaterniond q(eulerRodriguez<double>(roll, pitch, yaw));
		return q;
	}

	/**
	 * Transfers a vector to  a quaternion
	 * @param state the state vector
	 * @param startindex the start index of the euler angles
	 * @return an eigen Quaterniond
	 *
	 * state_dim dimension of the state
	 */
	template<int state_dim>
	inline ::Eigen::Quaterniond stateToQuaternion(Eigen::Matrix<double, state_dim, 1> & state, int startindex) {
		return ::Eigen::Quaterniond(state(startindex + 0), state(startindex + 1), state(startindex + 2),
				state(startindex + 3));
	}

	/**
	 * Object to call a function
	 *
	 * implements operator() to be called like a function and return a Eigen::Vector instead of a template pointer (ceres requirement)
	 *
	 * Implements an implicit type erasure so every function which needs a FuncCaller will accept snx objrvz ehivh implements the function (double * time, double * result, bool expand_noise)
	 */
	class FuncCaller {
	private:

		struct TypeErasure {
			virtual ~TypeErasure() {}
			virtual Eigen::Vector3d operator()(double time, bool expand_noise = false)const =0;

		};
		template<typename functor>
		struct FunctionWrapper: public TypeErasure {
			FunctionWrapper(const functor &function):function(function) {}

			virtual Eigen::Vector3d operator()(double time, bool expand_noise = false) const {
				double result[3];
				function(&time, result, expand_noise);
				return Eigen::Vector3d(result[0], result[1], result[2]);
			}
			functor function;
		};
	public:
		/**
		 * Calls the stored functor with parameters
		 * @param time time_point on wich the function es evaluated
		 * @param expand_noise whether to add a noisy increment
		 * @return a Eigen 3D Vector as the function result
		 */
		Eigen::Vector3d operator()(double time, bool expand_noise = false) const {
			return (*function)(time,expand_noise);
		}
		/**
		 * Create the funcCaller from an arbitrary object which implements the operator()((double * time, double * result, bool expand_noise)
		 * @param func function object
		 */
		template<typename functor>
		FuncCaller(const functor & func) :
		function(new FunctionWrapper<functor>(func)) {
		}
		/**
		 * Copy constructor
		 * @param func another FuncCaller
		 */
		FuncCaller(FuncCaller & func):
		function(func.function) {
		}

	protected:
		std::shared_ptr<TypeErasure> function;
	};

	/**
	 * First order numerical derivative
	 * @param time the time point
	 * @param function FunctionCaller   to derive
	 * @param itv the intervall for numeric derivation
	 * @return first order derivate at time
	 */

	inline Eigen::Vector3d diff1(double time,const FuncCaller & function, double itv = 1e-6) {
		return (function(time) - function(time - itv)) / itv;
	}
	/**
	 * Second order numerical derivative
	 * @param time the time point
	 * @param function  FunctionCaller   to derive
	 * @param itv the intervall for numeric derivation
	 * @return second order derivate at time
	 */
	inline Eigen::Vector3d diff2(double time,const FuncCaller & function, double itv = 1e-6) {
		return (diff1(time, function, itv) - diff1(time - itv, function, itv)) / itv;
	}
	/**
	 * Calculates the euler rotation angles of a function at a given time
	 *
	 * taken from https://stackoverflow.com/questions/18558910/direction-vector-to-rotation-matrix
	 *
	 * @param time  the time point
	 * @param function FunctionCaller   to calculate the orientation off
	 * @param itv the intervall for derivation purposes
	 * @param roll_function a function to determine roll, if not given roll is 0
	 * @return a vector with the 3 euler angles representing the orientation
	 */
	inline Eigen::Matrix3d orientationFromFunctor(double time,const FuncCaller & function, double itv,
			FuncCaller * ref_function = NULL) {
		Eigen::Vector3d vector_orient = diff1(time, function, itv);
		vector_orient.normalize();
		Eigen::Vector3d ref_axis =
		ref_function == NULL ? Eigen::Vector3d(0, 0, 1) : Eigen::Vector3d(ref_function->operator ()(time));
		Eigen::Vector3d xaxis = ref_axis.cross(vector_orient);
		xaxis.normalize();
		Eigen::Vector3d yaxis = vector_orient.cross(xaxis);
		yaxis.normalize();
		Eigen::Matrix3d orient;
		orient.col(1) = xaxis;
		orient.col(2) = yaxis;
		orient.col(0) = vector_orient;
		assert(abs(orient.determinant()-1)< 1e-3);
		return orient;
	}
	/**
	 * Wraps a rotation delta ( x,y,z) to -M_PI  + M_PI
	 * @param delta the vector to wrap
	 * @return a delta vector with values between -M_PI and + M_PI
	 */
	template<typename T>
	Eigen::Matrix<T,3,1> wrapRotDeltaVector(const Eigen::Matrix<T,3,1> & delta) {
		T sqnorm=delta.squaredNorm();
		if(sqnorm==T(0.)) {
			return delta;
		}
		T norm=sqrt(sqnorm);
		if(norm > M_PI) {
			T wrapped_norm=zavi::eigen_util::wrapAngle(norm);
			return delta*wrapped_norm/norm;
		}
		else
		return delta;
	}
	/**
	 * make s skew symmetric from w so that Q'=S(w)*Q  where Q is a rotation matrix and w its change
	 * @param w a 3 element vector
	 * @return a skew symmetric matrix of w
	 */
	template<typename T>
	Eigen::Matrix<T,3,3> makeSkewSymmetric(const Eigen::Matrix<T,3,1> & w) {
		Eigen::Matrix<T,3,3> S=S.Zero();
		S(0,1)=-w(2,0);
		S(0,2)=w(1,0);
		S(1,2)=-w(0,0);

		S(1,0)=w(2,0);
		S(2,0)=-w(1,0);
		S(2,1)=w(0,0);
		return S;
	}
	/**
	 * Determine whether a line intersects a plane
	 *
	 * where the line is l+t*l_d
	 *
	 * and the plane is p+u*p_d1 +v*p_d2
	 *
	 * with t,u,v variables in [0,1]
	 *
	 * length of vectors determines the line length / plane width/height
	 *
	 * @param line_base line start point
	 * @param line_dir  line direction vector
	 * @param plane_base plane start point
	 * @param plane_dir1  first plane direction vector
	 * @param plane_dir2  second plane direction vector
	 * @return
	 */
	template<typename T>
	bool lineIntersectsPlane(const Eigen::Matrix<T,3,1> &line_base,const Eigen::Matrix<T,3,1> &line_dir, const Eigen::Matrix<T,3,1> &plane_base, const Eigen::Matrix<T,3,1> &plane_dir1,const Eigen::Matrix<T,3,1> &plane_dir2 ) {
		Eigen::Matrix<T,3,3> A;
		A << -line_dir,plane_dir1 , plane_dir2;
		if(A.determinant() == 0.) {
			LOG(WARNING)<< "Unhandled Special case in lineIntersectsPlane. Line may be inside plane.";
			return false;
		}
		Eigen::Matrix<T,3,1> tuv=A.inverse()*(line_base-plane_base);
		bool intersects=true;
		for(unsigned int i=0; i < 3; i++) {
			if(tuv(i) < T(0.) or tuv(i) > T(1.))
			intersects=false;
		}
		return intersects;
	}

	/**
	 * For use from other functions
	 * @param a 3d Orientation matrix
	 * @param b 3d orientation matrix
	 * @return b boxminus a
	 */
	template<typename T, typename T2>
	inline static auto boxMinusOrientation(const Eigen::Matrix<T, 3, 3> &a,
			const Eigen::Matrix<T2, 3, 3> &b) ->Eigen::Matrix<decltype(a(0,0)*b(0,0)),3,1> {
		Eigen::Matrix<decltype(a(0,0)*b(0,0)), 3, 3> product= a.inverse() * b;
		return zavi::eigen_util::inverseEulerRodriguez<decltype(a(0,0)*b(0,0))>(product);
	}

	/**
	 * For use from other functions
	 * @param a Angle Axis vector
	 * @param b Angle Axis vector
	 * @return b boxminus a
	 */
	template<typename T, typename T2>
	inline static auto boxMinusEuler(const Eigen::Matrix<T, 3, 1> &a,
			const Eigen::Matrix<T2, 3, 1> &b) ->Eigen::Matrix<decltype(a(0,0)*b(0,0)),3,1> {
		return wrapAngles(b-a);
	}

	/**
	 * For use from other functions
	 * @param a Angle Axis vector
	 * @param b Angle Axis vector
	 * @return a boxplus b
	 */
	template<typename T, typename T2>
	inline static auto boxPlusEuler(const Eigen::Matrix<T, 3, 1> &state,
			const Eigen::Matrix<T2, 3, 1> &delta) ->Eigen::Matrix<decltype(state(0,0)*delta(0,0)),3,1> {
		return wrapAngles(state+delta);
	}



	/**
	 * boxplus operator for rotations
	 * @param state  the rotation matrix
	 * @param delta  the rotation change axis angle
	 * @return state boxplus delta
	 */
	template<typename T>
	inline static Eigen::Matrix<T, 3, 3> boxPlusOrientation(const Eigen::Matrix<T, 3, 3> &state,
			const Eigen::Matrix<T, 3, 1> &delta) {
		//assert(abs(state.determinant() - T(1.)) < T(1e-2));
		Eigen::Matrix<T, 3, 3> product = state* eigen_util::eulerRodriguez(wrapRotDeltaVector(delta));
		//assert(abs(product.determinant() - T(1.)) < T(1e-6));
		return product;

	}

	inline static Eigen::Matrix3d normaliseRotation(const Eigen::Matrix3d & rotation) {
		Eigen::Matrix3d Q= rotation.householderQr().householderQ();
		for(int i=0; i < 3 ; i ++){
			if(Q(i,i)<0.){
				Q.col(i)=-(Q.col(i));
			}
		}
		return Q;
	}

	inline double norm(const double a, const double b, const double c) {
		return sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));
	}

	template <typename T, int N> inline
	ceres::Jet<T, N> norm(const ceres::Jet<T, N>& a, const ceres::Jet<T, N>& b, const ceres::Jet<T, N>& c) {
		ceres::Jet<T, N> out;

		T const temp1 =sqrt(pow(a.a, 2) + pow(b.a, 2) + pow(c.a, 2));
		T const multiplier= 1./(temp1);
		T const temp2 = temp1==T(0.)? T(1./sqrt(3.)): multiplier*(a.a);
		T const temp3 = temp1==T(0.)? T(1./sqrt(3.)): multiplier*(b.a);
		T const temp4 = temp1==T(0.)? T(1./sqrt(3.)): multiplier*(c.a);

		out.a = temp1;
		out.v = temp2 * a.v + temp3 * b.v+temp4*c.v;
		return out;
	}

}
//zavi::eigen_util

#include "Eigen_Utils.tpp"
#endif /* EIGEN_UTILS_HPP_ */
