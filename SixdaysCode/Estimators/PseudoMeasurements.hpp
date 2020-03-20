/*
 * PseudoMeasurements.hpp
 *
 *  Created on: 02.08.2018
 *      Author: tomlucas
 */

#ifndef ESTIMATORS_PSEUDOMEASUREMENTS_HPP_
#define ESTIMATORS_PSEUDOMEASUREMENTS_HPP_

#include <Eigen/Core>

#include <glog/logging.h>
#include <ceres/ceres.h>

#include <Eigen_Utils.hpp>
#include "BatchEstimator.hpp"


/**
 * Setup a basic pseudo measurement so that other parts of the programm know its  dimensions
 */
#define SETUP_PSEUDO_MEASUREMENT(MEASURE_DIM)  \
	static constexpr int measure_dim=MEASURE_DIM; \
	static constexpr int state_dim=estimator_type::MODEL_TYPE::outer_size; \
	template<typename T>										\
	using MEASURE_TYPE=Eigen::Matrix<T,measure_dim,1>;

/**
 * Setup a pseudo measurement which contains a manifold with different inner size
 */
#define SETUP_PSEUDO_MEASUREMENT_WITH_MANIFOLD(MEASURE_DIM,MANIFOLD_DIM)  \
	SETUP_PSEUDO_MEASUREMENT(MEASURE_DIM)   \
	static constexpr int manifold_dim=MANIFOLD_DIM;   \
	template<typename T>										\
	using MANIFOLD_TYPE=Eigen::Matrix<T,manifold_dim,1>;

/**
 * Adds the necesarry function so that the UKF accepts the knowledge (for knowledge where alignment is not required)
 */
#define ADD_UKF_FUNCTION_PASSER  \
template<typename T>		\
MEASURE_TYPE<T> operator()(const Eigen::Matrix<T,state_dim,1> &state,const ALIGNMENT_TYPE<T> &alignment, void * prior) const {		\
			return this->operator()(state,prior);     \
		}

namespace zavi
::estimator::pseudo_measurement {

	/**
	 * Struct to contain relevant data for a function as Prior Knowledge
	 */
	template<typename ukf_type>
	struct FunctionMeasurement {
		FunctionMeasurement(const zavi::eigen_util::FuncCaller &function,std::shared_ptr<ukf_type> estimator ):function(function),ukf(estimator) {}
		zavi::eigen_util::FuncCaller function;
		std::shared_ptr<ukf_type> ukf;
	};
	/**
	 * pseudo measurement to return the position
	 * @param state  the current estimated state
	 * @param prior a maybe usefull prior
	 * @return an expected position at state
	 */
	inline Eigen::Matrix<double, 3, 1> position_measurement(const Eigen::Matrix<double,9,1> &state, void *prior) {
		return state.block(0,0,3,1);
	}


	/**
	 * Functor to calculate the shortest distance between a function and a point
	 */
	template <typename functor>
	class DistanceFunctor {
	public:
		~DistanceFunctor() {}
		DistanceFunctor(functor & function):function(function),x(0),y(0),z(0) {};
		template<typename T>
		bool operator()(const T * const time,T* residual) const {
			T func[3];
			function(time,(T* )func);
			residual[0]=pow(x-func[0],2)+pow(y-func[1],2)+pow(z-func[2],2);

			return true;
		}

	public:
		/**
		 * Set the point to which to calculate the distance
		 * @param point  point with x y z coordinates
		 */
		void setPoint(const Eigen::Matrix<double,3,1> & point) {
			x=point(0,0);
			y=point(1,0);
			z=point(2,0);
		}

		functor function;     // evaluation function
	private:

		double x,y,z;// point coordinates
	};

	/**
	 * Struct to contain relevant data for a function as Prior Knowledge
	 */
	template <typename functor,typename cov_type,typename ukf_type>
	struct DistanceMeasurement {
		DistanceMeasurement(functor &function,const cov_type &cov, std::shared_ptr<ukf_type> estimator ):function(function),cov(cov),ukf(estimator) {}
		DistanceFunctor<functor> function;
		cov_type cov;
		std::shared_ptr<ukf_type> ukf;
	};
	/**
	 * pseudo measurement to return the distance to prior function
	 * @param state  the current estimated state
	 * @param prior a maybe usefull prior
	 * @return an expected distance to function at state hard constriaint so it is always zero
	 */
	template <typename functor,typename ukf_type>
	Eigen::Matrix<double, 3, 1> distance_measurement(const Eigen::Matrix<double,9,1> &state, void * prior) {
		DistanceMeasurement<functor,Eigen::Matrix3d,ukf_type> * container= static_cast<DistanceMeasurement<functor,Eigen::Matrix3d,ukf_type> *>(prior);
		container->function.function.give_real=false;     // tell function that the solver is active
		// The variable to solve for with its initial value.
		static double initial_x = 0;
		double x = initial_x;
		container->function.setPoint(state.block(0,0,3,1));
		// Build the problem.
		// Set up the only cost function (also known as residual). This uses
		// auto-differentiation to obtain the derivative (jacobian).
		ceres::CostFunction* cost_function =
		new ceres::AutoDiffCostFunction<DistanceFunctor<functor>, 1, 1>(new DistanceFunctor<functor>(container->function));
		ceres::Problem problem;

		problem.AddResidualBlock(cost_function, NULL, &x);

		// Run the solver!
		ceres:: Solver::Options options;
		options.linear_solver_type = ceres::DENSE_QR;
		options.minimizer_progress_to_stdout = false;
		options.max_num_line_search_step_size_iterations=10;
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);
		//std::cout << summary.BriefReport() << std::endl;
		initial_x=x;
		double result[3];
		container->function.function(&x,(double *)result);
		Eigen::Matrix<double,3,1> returner;
		returner(0,0)=result[0]-state(0,0);
		returner(1,0)=result[1]-state(1,0);
		returner(2,0)=result[2]-state(2,0);

		container->function.function.give_real=true;// tell function that the solver is finished
		return returner;

	}
	/**
	 * pseudo measurement to return the distance to prior function
	 * @param state  the current estimated state
	 * @param prior a maybe usefull prior
	 * @return an expected distance to function at state hard constriaint so it is always zero
	 */
	template <typename functor,typename ukf_type>
	Eigen::Matrix<double, 1, 1> distance_measurement_single(const Eigen::Matrix<double,9,1> &state, void * prior) {
		DistanceMeasurement<functor,Eigen::Matrix<double,1,1>,ukf_type> * container= static_cast<DistanceMeasurement<functor,Eigen::Matrix<double,1,1>,ukf_type> *>(prior);
		container->function.function.give_real=false;     // tell function that the solver is active
		// The variable to solve for with its initial value.
		static double initial_x = 0;
		double x = initial_x;
		container->function.setPoint(state.block(0,0,3,1));
		// Build the problem.
		// Set up the only cost function (also known as residual). This uses
		// auto-differentiation to obtain the derivative (jacobian).
		ceres::CostFunction* cost_function =
		new ceres::AutoDiffCostFunction<DistanceFunctor<functor>, 1, 1>(new DistanceFunctor<functor>(container->function));
		ceres::Problem problem;

		problem.AddResidualBlock(cost_function, NULL, &x);

		// Run the solver!
		ceres:: Solver::Options options;
		options.linear_solver_type = ceres::DENSE_QR;
		options.minimizer_progress_to_stdout = false;
		options.max_num_line_search_step_size_iterations=10;
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);
		//std::cout << summary.BriefReport() << std::endl;
		initial_x=x;
		double result[1];
		container->function(&x,(double *)result);
		Eigen::Matrix<double,1,1> returner;
		returner(0,0)=result[0];

		container->function.function.give_real=true;// tell function that the solver is finished
		return returner;

	}

	/**
	 *  a pseudo distance measurement with a given function as ground truth
	 * @param plug  the imu plugin
	 * @param estimator  the function measurement struct
	 * @param time  time current time stamp
	 */
	template<typename functor,typename ukf_type>
	void pseudoDistanceFunctionMeasurement(plugin::SensorPlugin * plug,
			void* estimator, double time) {
		static double last_time=time-0.00001;
		DistanceMeasurement<functor,Eigen::Matrix3d,ukf_type> * container= static_cast<DistanceMeasurement<functor,Eigen::Matrix3d,ukf_type> *>(estimator);
		//plugin::IMU_Plugin *imu = static_cast<plugin::IMU_Plugin*>(plug);
		Eigen::Matrix<double,3,1> zeros=Eigen::Matrix<double,3,1>::Zero();
		container->ukf->measurementStep(zeros,time-last_time,distance_measurement<functor>,container->cov,container);
		last_time=time;
	}

	/**
	 *  a pseudo distance measurement with a given function as ground truth
	 * @param plug  the imu plugin
	 * @param estimator  the function measurement struct
	 * @param time  time current time stamp
	 */
	template<typename functor,typename ukf_type>
	void pseudoDistanceFunctionMeasurementSingle(plugin::SensorPlugin * plug,
			void* estimator, double time) {
		static double last_time=time-0.00001;
		DistanceMeasurement<functor,Eigen::Matrix<double,1,1>,ukf_type> * container= static_cast<DistanceMeasurement<functor,Eigen::Matrix<double,1,1>,ukf_type> *>(estimator);
		//plugin::IMU_Plugin *imu = static_cast<plugin::IMU_Plugin*>(plug);
		Eigen::Matrix<double,1,1> zeros=Eigen::Matrix<double,1,1>::Zero();
		container->ukf->measurementStep(zeros,time-last_time,distance_measurement_single<functor>,container->cov,container);
		last_time=time;
	}



	/**
	 * This Struct encapsulates the driving forward constrain
	 * The Velocity in y and z direction ( in body coordinates ) is 0
	 * The Velocity in x must equal the norm of the velocity
	 *
	 * It requires the estimator as prior
	 */
	template<typename estimator_type>
	struct DrivingForwardConstrain {
		SETUP_PSEUDO_MEASUREMENT(3)
		template<typename T>
		MEASURE_TYPE<T> operator()(const Eigen::Matrix<T,state_dim,1> &state,void * prior) const {
			estimator_type * estimator=static_cast<estimator_type*>(prior);
			MEASURE_TYPE<T> body_velocity=estimator->getBoxModel().getOrientation(state).transpose()*estimator->getBoxModel().getVelocity(state)-getTarget(state,prior);
			return body_velocity;

		}
		ADD_UKF_FUNCTION_PASSER
		template<typename T>
		MEASURE_TYPE<T> getTarget(const Eigen::Matrix<T,state_dim,1> &state, void * prior) const {
			estimator_type * estimator=static_cast<estimator_type*>(prior);
			MEASURE_TYPE<T> returner=MEASURE_TYPE<T>::Zero();
			T norm=estimator->getBoxModel().getVelocity(state).norm();
			if(norm !=0.)     // ceres jet infinity fix
			returner(0,0)=norm;
			return returner;

		}


		template<typename T>
		static constexpr MEASURE_TYPE<T> getTarget() {
			return MEASURE_TYPE<T>::Zero();
		}
		static Eigen::Matrix<double,measure_dim,measure_dim> getNoise() {
			return Eigen::Matrix<double,measure_dim,measure_dim>::Identity()*0.1;
		}
	};

	/**
	 * This Struct encapsulates the driving forward constrain
	 * The Velocity in y and z direction ( in body coordinates ) is 0
	 *
	 * It requires the estimator as prior
	 */
	template<typename estimator_type>
	struct NoSideUpDriftConstrain {
		SETUP_PSEUDO_MEASUREMENT(2)
		template<typename T>
		MEASURE_TYPE<T> operator()(const Eigen::Matrix<T,state_dim,1> &state, void * prior) const {
			return ( (estimator_type::MODEL_TYPE::getOrientation(state).transpose()*estimator_type::MODEL_TYPE::getVelocity(state))) .template block<2,1>(1,0);

		}
		ADD_UKF_FUNCTION_PASSER
		template<typename T>
		static constexpr MEASURE_TYPE<T> getTarget() {
			return MEASURE_TYPE<T>::Zero();
		}
		static Eigen::Matrix<double,measure_dim,measure_dim> getNoise() {
			return Eigen::Matrix<double,measure_dim,measure_dim>::Identity()*0.1;
		}
	};

	template<typename estimator_type>
	struct OnlyRollConstrain {
		static constexpr int measure_dim=2;     // measurement dimension
		static constexpr int state_dim=estimator_type::MODEL_TYPE::outer_size;
		template<typename T>
		using MEASURE_TYPE=Eigen::Matrix<T,measure_dim,1>;
		template<typename T>
		MEASURE_TYPE<T> operator()(const Eigen::Matrix<T,state_dim,1> &state, void * prior) const {
			return eigen_util::inverseEulerRodriguez<T>(estimator_type::MODEL_TYPE::getOrientation(state)).template block<2,1>(1,0);

		}

		ADD_UKF_FUNCTION_PASSER
		template<typename T>
		static constexpr MEASURE_TYPE<T> getTarget() {
			return MEASURE_TYPE<T>::Zero();
		}
		static Eigen::Matrix<double,measure_dim,measure_dim> getNoise() {
			return Eigen::Matrix<double,measure_dim,measure_dim>::Identity()*0.01;
		}
	};

	/**
	 * This Struct encapsulates the driving forward constrain
	 * The Velocity in y and z direction ( in body coordinates ) is 0
	 *
	 * It requires the estimator as prior
	 */
	template<typename estimator_type, typename functor>
	struct FunctionOrientationMeasurement {

		SETUP_PSEUDO_MEASUREMENT_WITH_MANIFOLD(3,3)

		eigen_util::FuncCaller function;
		FunctionOrientationMeasurement<estimator_type,functor>() :function(functor()) {

		}

		template<typename T>
		MEASURE_TYPE<T> operator()(const Eigen::Matrix<T,state_dim,1> &state, void * prior) const {
			estimator_type* estimator= static_cast<estimator_type *>(prior);
			return eigen_util::boxMinusOrientation(estimator->getBoxModel().getOrientation(state),eigen_util::orientationFromFunctor(state(estimator->getBoxModel().lambda_start,0),function,1e-5));

		}
		ADD_UKF_FUNCTION_PASSER

		template<typename T>
		static MANIFOLD_TYPE<T> boxminus(const MEASURE_TYPE<T> &a , const MEASURE_TYPE<T> &b) {
			return eigen_util::wrapAngles(b-a);
		}
		template<typename T>
		static MEASURE_TYPE<T> boxplus(const MEASURE_TYPE<T> &state , const MANIFOLD_TYPE<T> &delta) {
			return eigen_util::wrapAngles(state+delta);
		}


		template<typename T>
		static constexpr MEASURE_TYPE<T> getTarget() {
			return MEASURE_TYPE<T>::Zero();
		}
		static Eigen::Matrix<double,measure_dim,measure_dim> getNoise() {
			return Eigen::Matrix<double,measure_dim,measure_dim>::Identity()*0.3;
		}
	};
	/**
	 * This Struct encapsulates the driving forward constrain
	 * The Velocity in y and z direction ( in body coordinates ) is 0
	 *
	 * It requires the estimator as prior
	 */
	template<typename estimator_type, typename functor>
	struct FunctionOrientationChangeMeasurement {
		SETUP_PSEUDO_MEASUREMENT(3)

		eigen_util::FuncCaller function;
		FunctionOrientationChangeMeasurement<estimator_type,functor>() :function(functor()) {

		}

		template<typename T>
		MEASURE_TYPE<T> operator()(const Eigen::Matrix<T,state_dim,1> &state, void * prior) const {
			estimator_type* estimator= static_cast<estimator_type *>(prior);
			Eigen::Matrix<T,3,1> current_axis=estimator->getBoxModel().getOrientationChange(state);
			Eigen::Matrix<T,3,1> target_axis=eigen_util::boxMinusOrientation(eigen_util::orientationFromFunctor(state(estimator->getBoxModel().lambda_start,0)-1e-5,function,1e-5),eigen_util::orientationFromFunctor(state(estimator->getBoxModel().lambda_start,0)+1e-5,function,1e-5))/2e-5;
			//zavi::plot::liveStateDraw<6>(target_axis,"target_axis");
			/*if(current_axis.norm()!=0.)
			current_axis.normalize();*/
			/*if(target_axis.norm() !=0.)
			target_axis.normalize();*/
			return current_axis-target_axis*estimator->getBoxModel().getLambdaVelocity(state);

		}
		ADD_UKF_FUNCTION_PASSER

		template<typename T>
		static constexpr MEASURE_TYPE<T> getTarget() {
			return MEASURE_TYPE<T>::Zero();
		}
		static Eigen::Matrix<double,measure_dim,measure_dim> getNoise() {
			return Eigen::Matrix<double,measure_dim,measure_dim>::Identity()*0.5;
		}
	};

	/**
	 * pseudo ukf measurement
	 * @template measure_struct a struct with () operator to define the measure function
	 * @template ukf_type the type of the estimator
	 * @template measure_dim the size of the measurement
	 * @template measurement a measurement object containing the wanted result
	 * @template cov the covariance of the pseudo measurement
	 */
	template<template<typename,typename ...> class measure_struct,typename ukf_type,typename ... meas_args>
	void ukfMeasurement(plugin::SensorPlugin * plug,
			void* estimator, double time) {
		static double last_time=time-0.00001;
		static measure_struct<ukf_type,meas_args ...> measure_function=measure_struct<ukf_type,meas_args ...>();
		ukf_type *ukf=static_cast<ukf_type*>(estimator);
		ukf->measurementStep(measure_function.template getTarget<double>(),time-last_time,measure_function,measure_function.getNoise(),estimator);     // No Prior is given since the circle_prior does not need an object
		last_time=time;
	}
	/**
	 * pseudo ukf measurement for manifolds
	 * @template measure_struct a struct with () operator to define the measure function
	 * @template ukf_type the type of the estimator
	 * @template measure_dim the size of the measurement
	 * @template measurement a measurement object containing the wanted result
	 * @template cov the covariance of the pseudo measurement
	 */
	template<template<typename,typename ...> class measure_struct,typename ukf_type,typename ... meas_args>
	void ukfManifoldMeasurement(plugin::SensorPlugin * plug,
			void* estimator, double time) {
		static double last_time=time-0.00001;
		typedef measure_struct<ukf_type,meas_args ...> MEASURE_STRUCT;
		static MEASURE_STRUCT measure_function=MEASURE_STRUCT();
		ukf_type *ukf=static_cast<ukf_type*>(estimator);
		//measure_function.template boxplus<double>(Eigen::Vector3d(), Eigen::Vector3d());
		ukf->measurementStepManifold(measure_function.template getTarget<double>(),time-last_time,measure_function,measure_function.getNoise(),MEASURE_STRUCT::boxplus,MEASURE_STRUCT::boxminus,estimator);
		last_time=time;
	}


	/**
	 * A cost function comparing the 1 state with the required circle trajectory
	 */
	template<template <typename T> class STATE_TYPE_T,typename MEASUREMENT_FUNCTION, int measure_dim >
	struct MeasurementDiffCostFunction {
		template<typename T>
		using MEASUREMENT_TYPE_T= Eigen::Matrix<T,measure_dim,1>;
		static inline MEASUREMENT_FUNCTION measurement_function=MEASUREMENT_FUNCTION();
		MeasurementDiffCostFunction(void * prior):prior(prior) {}

		template<typename T>
		bool operator()(const T * const a,const T* const alignment, T* result)const {
			static Eigen::Matrix<double,measure_dim,measure_dim> stiffnes=measurement_function.getNoise().diagonal().cwiseInverse().cwiseSqrt().asDiagonal();
			static Eigen::Matrix<double,measure_dim,1> measurement=measurement_function.template getTarget<double>();
			(Eigen::Map<MEASUREMENT_TYPE_T<T> > (result))=stiffnes*(measurement_function.template operator()<T>(Eigen::Map<const STATE_TYPE_T<T> >(a),Eigen::Map<const ALIGNMENT_TYPE<T> >(alignment),prior)-measurement);
			return true;
		}

		template<typename T>
		bool operator()(const T * const a, T* result)const {
			static Eigen::Matrix<double,measure_dim,measure_dim> stiffnes=measurement_function.getNoise().diagonal().cwiseInverse().cwiseSqrt().asDiagonal();
			static Eigen::Matrix<double,measure_dim,1> measurement=measurement_function.template getTarget<double>();
			(Eigen::Map<MEASUREMENT_TYPE_T<T> > (result))=stiffnes*(measurement_function.template operator()<T>(Eigen::Map<const STATE_TYPE_T<T> >(a),prior)-measurement);
			return true;
		}
		void * prior;
	};
	/**
	 * Adds the circular constraint to the ceres problem
	 * @param problem  ceres problem
	 * @param states list of all states
	 * @param inputs list of all inputs
	 * @param time_diffs list of all time diffs
	 */

	template<template<typename> class MEASUREMENT_FUNCTION, typename ESTIMATOR_TYPE,bool with_alignment=false>
	struct MeasurementConstraint: public ESTIMATOR_TYPE::BatchConstrain {
		static constexpr int measure_dim=MEASUREMENT_FUNCTION<ESTIMATOR_TYPE>::measure_dim;
		virtual ~MeasurementConstraint() {};
		typedef MeasurementDiffCostFunction<ESTIMATOR_TYPE::template STATE_TYPE_T,MEASUREMENT_FUNCTION<ESTIMATOR_TYPE>,measure_dim > COST_TYPE;
		virtual void operator()(ceres::Problem &problem,double * states,double * alignment, int state_size,void* data,std::vector<double> &time_diffs, BatchEstimator<typename ESTIMATOR_TYPE::MODEL_TYPE> * estimator) {
			if(with_alignment) {
				ceres::CostFunction* cost_function=new ceres::AutoDiffCostFunction<COST_TYPE,measure_dim,ESTIMATOR_TYPE::MODEL_TYPE::outer_size, ALIGNMENT_SIZE>(new COST_TYPE(estimator));
				for(int i=0; i < state_size;i++) {
					problem.AddResidualBlock(cost_function,NULL,&states[i*ESTIMATOR_TYPE::MODEL_TYPE::outer_size], alignment);
				}
			}
			else {
				ceres::CostFunction* cost_function=new ceres::AutoDiffCostFunction<COST_TYPE,measure_dim,ESTIMATOR_TYPE::MODEL_TYPE::outer_size>(new COST_TYPE(estimator));
				for(int i=0; i < state_size;i++) {
					problem.AddResidualBlock(cost_function,NULL,&states[i*ESTIMATOR_TYPE::MODEL_TYPE::outer_size]);
				}
			}
		}
	};

}
//zavi::estimator::pseudo_measurement

#endif /* ESTIMATORS_PSEUDOMEASUREMENTS_HPP_ */
