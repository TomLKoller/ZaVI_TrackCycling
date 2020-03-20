/*
 * BatchEstimator.hpp
 *
 *  Created on: 03.09.2018
 *      Author: tomlucas
 */

#ifndef ESTIMATORS_BATCHESTIMATOR_HPP_
#define ESTIMATORS_BATCHESTIMATOR_HPP_

#include "../Plugins/imu_plugin.hpp"
#include "../OSG_VIZ/osg_viz.hpp"
#include <OSG_Utils.hpp>
#include "AlignType.hpp"

#include <ZaviConfig.hpp>
#include <Eigen/Core>
#include <ceres/ceres.h>

#include <ceres/autodiff_local_parameterization.h>
#include <ceres/local_parameterization.h>
#include <ceres/loss_function.h>
#include <vector>
#include <limits>
#include <Callbacker.hpp>
#include "Models/State.hpp"
#include "StateBoxes/HomogeneousCoordinatesBox.hpp"
namespace zavi
::estimator {
#define CARTESIAN_DIMENSIONS 3
	typedef model::State<state_boxes::HomoCoordBox> ALIGNMENT_PARAMETERIZATION;

	/**
	 * This class implements the basic functionality of a batch estimator
	 *
	 * constraints (measurements) have to be added in derivates of this class via addConstraint
	 */
	template<typename model_type>
	class BatchEstimator: public Estimator {
	protected:

		/**
		 * Create the plot to show the estimated trajectory this is basic functionality since the batch estimator shouldnt be called at every iteration for performance reasons
		 * instead it shows the past trajectory as a state plot
		 */
		BatchEstimator(std::shared_ptr<plugin::SensorPlugin> sensor, bool output_replay = false, double skip_time=0) :
		box_model(sensor), output_replay(output_replay), current_estimation(NULL), current_replay_index(-1), current_size(
				0), states(NULL), alignment(NULL), align_parameter(sensor) , skip_time(skip_time),num_states(0) {
			//plot=zavi::plot::createPlot();
			state_vector = state_vector.Zero();
			lower_bound = lower_bound.Ones() * (0 - std::numeric_limits<double>::infinity());
			upper_bound = upper_bound.Ones() * std::numeric_limits<double>::infinity();
			if (output_replay)
			Timer::activateReplay(true);
		}
	public:
		// List of callback indices
		enum CallbackIndices {CI_ESTIMATE_SUBSEQUENCE_READY,     //!< called after every subsequence (first 15k states, first 25k states...)
			CI_EVERY_SOLVER_ITERATION_CALLBACK,//!< called at every ceres iteration callback
			CI_REPLAY_COUNTER_INCREASE,//!< called whenever the counter for replay of states is increased
			CI_FAST_REPLAY_ITERATION,//!< called in every iteration of fast replay
			CI_ESTIMATE_READY};     //!< called when estimate is completed

		typedef model_type MODEL_TYPE;//model type
		template<typename T>
		using STATE_TYPE_T= typename MODEL_TYPE::template OUTER_T<T>;
		template<typename T>
		using SIGMA_TYPE_T= typename MODEL_TYPE::template INNER_T<T>;
		template<typename T>
		using INPUT_TYPE_T= typename MODEL_TYPE::template INPUT_T<T>;
		virtual ~BatchEstimator() {
			if (current_estimation != NULL)
			delete current_estimation;
			if (alignment != NULL)
			delete alignment;

		}

		/**
		 * Callback for imu
		 * @param plug the imu plugin which has new data
		 * @param estimator the estimator which gets the new data
		 * @param time the current time stamp
		 */
		static void batchCallback(plugin::SensorPlugin * plug, void* estimator, double time) {
			BatchEstimator * esti = static_cast<BatchEstimator *>(estimator);
			esti->estimate();
		}

		/**
		 *
		 * @param start start point for transition
		 * @param next  after the transition
		 * @param time_diff time_diff between points
		 * @param next_index the index to take readings of input
		 */
		virtual void advanceState(double * start, double * next, double time_diff, double next_index)=0;

		virtual void setInitialState(double * data, unsigned int index)=0;
		/**
		 * Set the pose in data to the middle of the constrained area
		 * @param data the pointer to the data
		 */
		virtual void setNeutralPose(double * data) {
			for (int i = 0; i < 3; i++)
			data[i] = lower_bound(i, 0) + (upper_bound(i, 0) - lower_bound(i, 0)) * 0.5;
		}
		/**
		 * Add the calibration matrix to the problem
		 * @param problem the problem to add the alignment to
		 * @param align  the alignment
		 * @param constant if constant is true, the estimator does not alter the alignment
		 */
		void addAlignment(ceres::Problem & problem,const double * align=NULL, bool constant = true) {
			ALIGNMENT_TYPE<double> temp = temp.Identity();
			alignment = (double *) malloc(ALIGNMENT_SIZE * sizeof(double));
			if(align !=NULL) {
				LOG(INFO) << "loading alignment";
				memcpy(alignment,align,ALIGNMENT_SIZE * sizeof(double));
				Eigen::Map<Eigen::Matrix4d> map(alignment);
				map.block<3,3>(0,0)=eigen_util::normaliseRotation(map.block<3,3>(0,0));
			}
			else {
				memcpy(alignment, temp.data(), sizeof(double) * ALIGNMENT_SIZE);
			}
			ceres::LocalParameterization * local_parameterization = new ceres::AutoDiffLocalParameterization<
			ALIGNMENT_PARAMETERIZATION, ALIGNMENT_PARAMETERIZATION::outer_size,
			ALIGNMENT_PARAMETERIZATION::inner_size>(new ALIGNMENT_PARAMETERIZATION(align_parameter));
			problem.AddParameterBlock(alignment, ALIGNMENT_PARAMETERIZATION::outer_size, local_parameterization);
			if (constant)
			problem.SetParameterBlockConstant(alignment);
		}

		/**
		 * Fullfills a batch estimate
		 */
		virtual void estimate() {
			if (inputs.size() == 0)
			return;     // nothing to do if no no constraints can be put
			double * temp = NULL;


			num_states = inputs.size();

			if (states == NULL)
			states = (double *) malloc(num_states * MODEL_TYPE::outer_size * sizeof(double));

			options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
			options.minimizer_progress_to_stdout = true;
			options.num_threads = 10;
			options.gradient_tolerance = 1.;
			options.max_num_iterations = 12;
			options.update_state_every_iteration = true;

			IterationCounter counter(states, num_states, this);
			options.callbacks.push_back(&counter);
			ceres::Solver::Summary summary;

			osg::ref_ptr<osg::Geode> geo(new osg::Geode());
			zavi::osg_viz::root->addChild(geo);
			std::printf("Creating %zu states\n", num_states);
			for (unsigned int i = 0; i < num_states; i++) {
				setInitialState(&states[MODEL_TYPE::outer_size * i], i);
			}
			bool final_iteration = false;
			unsigned int last_state_size = 0;

			for (unsigned int state_size = 15000; !final_iteration; state_size += 2500) {

				ceres::LocalParameterization * local_parameterization = new ceres::AutoDiffLocalParameterization<MODEL_TYPE,
				MODEL_TYPE::outer_size, MODEL_TYPE::inner_size>(new MODEL_TYPE(box_model));
				if (state_size >= num_states) {
					state_size = num_states;
					final_iteration = true;
					options.gradient_tolerance = 1e-10;
					options.max_num_iterations = 50;
				}

				ceres::Problem problem;
				addAlignment(problem);
				for (unsigned int i = 0; i < state_size; i++) {
					if (last_state_size > 0 and i >= last_state_size)
					advanceState(&states[(i - 1) * MODEL_TYPE::outer_size], &states[i * MODEL_TYPE::outer_size],
							time_diffs[i - 1], i);

					//setNeutralPose(&states[i*MODEL_TYPE::outer_size]);

					problem.AddParameterBlock(&states[i * MODEL_TYPE::outer_size], MODEL_TYPE::outer_size,
							local_parameterization);
					for (int k = 0; k < MODEL_TYPE::outer_size; k++) {
						problem.SetParameterLowerBound(&states[i * MODEL_TYPE::outer_size], k, lower_bound(k, 0));
						problem.SetParameterUpperBound(&states[i * MODEL_TYPE::outer_size], k, upper_bound(k, 0));
					}
				}
				addConstraints(problem, states, state_size, this);

				printf("Starting solver\n");
				ceres::Solve(options, &problem, &summary);
				std::cout << summary.FullReport() << "\n";
				printf("Estimate complete\n");
				//zavi::plot::stateDraw<0>(states,"Batch estimated states");
				if (output_replay) {
					temp = (double *) current_estimation;     // safe pointer to delete old data
					current_estimation = (double *) malloc(state_size * MODEL_TYPE::outer_size * sizeof(double));
					current_size = state_size;
					memcpy((void *) current_estimation, states, state_size * MODEL_TYPE::outer_size * sizeof(double));
					if (temp != NULL and temp != states)
					delete temp;
					std::vector<STATE_TYPE_T<double>> all_states;
					std::vector<double> norms;
					//auto input_measurement=typename MODEL_TYPE::input_measurement();
					double max_norm = 0.;
					for (unsigned int i = 0; i < state_size; i++) {
						all_states.push_back(STATE_TYPE_T<double>(&states[i * MODEL_TYPE::outer_size]));
						//norms.push_back((input_measurement.template operator()<double>(all_states.back(),Eigen::Map<Eigen::Matrix<double,4,4> >(alignment),NULL)-inputs[i]).norm());
						if (norms.back() > max_norm)
						max_norm = norms.back();
					}

					for (unsigned int i = 0; i < state_size; i++) {
						norms[i] = norms[i] / max_norm;
					}
					zavi::printf("First state is :");
					zavi::printf(all_states[0]);
					if (geo->getNumChildren() > 0) {
						geo->removeChild(0, 1);
					}
					geo->addChild(
							zavi::osg_viz::pathShower((double *) current_estimation, &norms[0], current_size,
									MODEL_TYPE::outer_size));
					//zavi::plot::stateDraw<0>(all_states,"Estimated States");
					//zavi::plot::stateDraw<1>(inputs,"INputs");

				} else {
					state_vector = STATE_TYPE_T<double>(&states[(state_size - 1) * MODEL_TYPE::outer_size]);
					delete states;
					delete alignment;
				}
				last_state_size = state_size;
				notifyCallbacks(CI_ESTIMATE_SUBSEQUENCE_READY);
			}
			notifyCallbacks(CI_ESTIMATE_READY);
		}
		/**
		 * Does a fast replay and calls
		 * callbacks under CI_FAST_REPLAY_ITERATION
		 *
		 * Inhibits normal replay and restarts it afterwards
		 *
		 * @param state_size the amount of states to replay
		 */
		void fastReplay(int state_size) {
			int temp_cur_size = current_size;
			current_size = 0;     // set 0 to not be intereferd by normal replay
			double time = skip_time;
			while (current_replay_index < state_size - 1) {
				current_replay_index++;
				time += time_diffs[current_replay_index];
				if (!current_replay_index)     // fix that first time diff is artificially 0.00001 or something like that
				time = skip_time;
				Timer::setReplayTime(time);
				notifyCallbacks(CI_FAST_REPLAY_ITERATION);
			}
			//restart normal replay
			current_size = temp_cur_size;
			current_replay_index = -1;
		}

		/**
		 * calls estimate with freq
		 * @param freq loop frequency
		 * @param running function to determine whether this thread should be running
		 */
		void run(double freq, bool (*running)()) {
			Timer loop(freq);
			double time = skip_time;
			while (running()) {
				if (output_replay) {
					if (current_replay_index < current_size - 1) {
						current_replay_index++;
						time += time_diffs[current_replay_index];
						if (!current_replay_index)     // fix that first time diff is artificially 0.00001 or something like that
						time = skip_time;
						Timer::setReplayTime(time);
						notifyCallbacks(CI_REPLAY_COUNTER_INCREASE);
					}
					loop.wait();
				} else {
					loop.wait();     // dont start estimating directly since nothing is there to estimate
					estimate();
					printf("Estimated values\n");
				}
			}

		}

		virtual inline Eigen::Vector3d getEstimatedPosition() {
			if (current_replay_index >= 0) {
				return Eigen::Map<Eigen::Vector3d>(
						(double *) &current_estimation[current_replay_index * MODEL_TYPE::outer_size]);
			} else
			return (state_vector.block(0, 0, 3, 1));
		}

		virtual inline Eigen::Quaterniond getEstimatedOrientation() {
			if (current_replay_index >= 0) {
				return Eigen::Quaterniond(
						MODEL_TYPE::template getOrientation<double>(
								Eigen::Map<STATE_TYPE_T<double> >(
										(double *) &current_estimation[current_replay_index * MODEL_TYPE::outer_size])));
			} else
			return Eigen::Quaterniond(Eigen::AngleAxisd(0, Eigen::Vector3d(1, 0, 0)));     //< static orientation
		}

		virtual inline Eigen::Matrix3d getEstimatedPositionError() {
			throw " Not implemented getEstimatedPositionError for BatchEstimator";
		}

		inline MODEL_TYPE getBoxModel() {
			return box_model;
		}
	protected:

		/**
		 * Callback to get the iteration count
		 */
		class IterationCounter: public ceres::IterationCallback {
		public:
			static constexpr int num_iterations_till_full = 20;
			static inline volatile int counter = 1;
			static double ratio() {
				return (double) counter / num_iterations_till_full;
			}
			double* states;
			int num_states;
			osg::ref_ptr<osg::Geode> geo;
			BatchEstimator * estimator;
			IterationCounter(double * states, int num_states, BatchEstimator * estimator) :
			states(states), num_states(num_states), geo(new osg::Geode()), estimator(estimator) {
				zavi::osg_viz::root->addChild(geo);

			}
			virtual ~IterationCounter() {
				zavi::printf("Deleting geometry of callback");
				zavi::osg_viz::root->removeChild(geo);
			}

			virtual ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
				counter = summary.iteration + 2;     // +2 because it starts at 0 but we assume 1 to be the first element and we get the number after and not before the iteration
				if (geo->getNumChildren() > 0) {
					geo->removeChild(0, 1);
				}
				geo->addChild(zavi::osg_viz::pathShower((double *) states, num_states, MODEL_TYPE::outer_size));

				for (unsigned int i = 0; i < num_states; i++) {
					Eigen::Map<STATE_TYPE_T<double> > state(&states[i * MODEL_TYPE::outer_size]);
					state=estimator->box_model.normalise(state);
				}

				estimator->notifyCallbacks(CI_EVERY_SOLVER_ITERATION_CALLBACK);
				return ceres::SOLVER_CONTINUE;
			}
		}
		;

		/**
		 * A cost function comparing the difference of 2 states with the corresponding measurement
		 *
		 * uses boxMinus of the Model
		 */
		struct TransitionCostFunction {
			double time_diff;     //< the timediff at this transition
			BatchEstimator *estimator;
			double ratio_index;//number of this constrain
			TransitionCostFunction(double time_diff, BatchEstimator *estimator, int index, int state_num) :
			time_diff(time_diff), estimator(estimator), ratio_index((double) index / state_num) {

			}

			double costModifier() const {
				return 1. + 1. / pow(7.5 * (ratio_index + IterationCounter::ratio()), 2);
			}

			template<typename T>
			bool operator()(const T * const a, const T* const b, T* result) const {
				(Eigen::Map<typename MODEL_TYPE::template COST_T<T> >(result)) =
				(this->estimator->getBoxModel().template transitionCost < T
						> (Eigen::Map<const STATE_TYPE_T<T> >(a), time_diff, Eigen::Map<const STATE_TYPE_T<T> >(
										b)));
				return true;
			}
		};
		/**
		 * applies the transition constraint
		 * @param problem  ceres problem
		 * @param states list of all states
		 * @param inputs list of all inputs
		 * @param time_diffs list of all time diffs
		 */
		static void transitionConstrain(ceres::Problem &problem, double *states, double * alignment, int state_size,
				void * data, std::vector<double> &time_diffs, BatchEstimator *estimator) {
			for (int i = 0; i < state_size - 1; i++) {
				ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<TransitionCostFunction,
				MODEL_TYPE::cost_size, MODEL_TYPE::outer_size, MODEL_TYPE::outer_size>(
						new TransitionCostFunction(time_diffs[i+1], estimator, i, state_size - 1));
				problem.AddResidualBlock(cost_function, NULL, &states[i * MODEL_TYPE::outer_size],
						&states[(i + 1) * MODEL_TYPE::outer_size]);
			}
		}
		/**
		 * A cost function comparing the 1 state with the required circle trajectory
		 */
		template<template<typename T> class STATE_TYPE_T, typename MEASUREMENT_FUNCTION, int measure_dim>
		struct StateDiffFunction {
			template<typename T>
			using MEASUREMENT_TYPE_T= Eigen::Matrix<T,measure_dim,1>;
			MEASUREMENT_TYPE_T<double> & measurement;
			BatchEstimator *estimator;
			static inline MEASUREMENT_FUNCTION measurement_function = MEASUREMENT_FUNCTION();
			StateDiffFunction(MEASUREMENT_TYPE_T<double> & measurement, BatchEstimator *estimator) :
			measurement(measurement), estimator(estimator) {
				for (int i = 0; i < measure_dim; i++) {
					assert(measurement(i,0)!=NAN);
				}
				assert(measurement_function.stiffnes.determinant() > 0.);
			}
			template<typename T>
			bool operator()(const T * const a, const T * const alignment, T* result) const {
				(Eigen::Map<Eigen::Matrix<T, MEASUREMENT_FUNCTION::output_size, 1> >(result)) =
				measurement_function.stiffnes
				* measurement_function.template boxminus<T>(
						measurement_function.template operator()<T>(
								Eigen::Map<const STATE_TYPE_T<T> >(a),
								Eigen::Map<const ALIGNMENT_TYPE<T> >(alignment), this->estimator),
						measurement);
				return true;
			}
		};

		/**
		 * A cost function comparing the 1 state with the required circle trajectory
		 */
		template<template<typename T> class STATE_TYPE_T, typename MEASUREMENT_FUNCTION, int measure_dim>
		struct InputDiffFunction {
			template<typename T>
			using MEASUREMENT_TYPE_T= Eigen::Matrix<T,measure_dim,1>;
			MEASUREMENT_TYPE_T<double> & measurement;
			BatchEstimator *estimator;
			static inline MEASUREMENT_FUNCTION measurement_function = MEASUREMENT_FUNCTION();
			InputDiffFunction(MEASUREMENT_TYPE_T<double> & measurement, BatchEstimator *estimator) :
			measurement(measurement), estimator(estimator) {
				for (int i = 0; i < measure_dim; i++) {
					assert(measurement(i,0)!=NAN);
				}
				assert(measurement_function.stiffnes.determinant() > 0.);
			}
			template<typename T>
			bool operator()(const T * const a, const T * const alignment, T* result) const {
				(Eigen::Map<Eigen::Matrix<T, MEASUREMENT_FUNCTION::output_size, 1> >(result)) =
				measurement_function.stiffnes
				* measurement_function.template boxminus<T>(
						measurement_function.template operator()<T>(
								Eigen::Map<const STATE_TYPE_T<T> >(a),
								Eigen::Map<const ALIGNMENT_TYPE<T> >(alignment), this->estimator),
						measurement);
				return true;
			}
		};

		/**
		 * A cost function comparing the 1 state with the required circle trajectory
		 */
		template<template<typename T> class STATE_TYPE_T, typename MEASUREMENT_FUNCTION, int measure_dim>
		struct LowInputFunction {
			template<typename T>
			using MEASUREMENT_TYPE_T= Eigen::Matrix<T,measure_dim,1>;
			BatchEstimator *estimator;
			static inline MEASUREMENT_FUNCTION measurement_function = MEASUREMENT_FUNCTION();
			LowInputFunction( BatchEstimator *estimator) :
			estimator(estimator) {
			}
			template<typename T>
			bool operator()(const T * const a, const T * const alignment, T* result) const {
				static Eigen::Matrix<double,6,6> stiffness=stiffness.Identity()*cfg->lookup("dummy_acc_stiffness");
				static bool firstcall=true;
				if(firstcall) {//really bad implementation
					stiffness.block<3,3>(3,3)=Eigen::Matrix3d::Identity()*cfg->lookup("dummy_ar_stiffness");
					firstcall=false;
				}
				(Eigen::Map<Eigen::Matrix<T, MEASUREMENT_FUNCTION::output_size, 1> >(result)) =
				stiffness*measurement_function.template operator()<T>(
						Eigen::Map<const STATE_TYPE_T<T> >(a),
						Eigen::Map<const ALIGNMENT_TYPE<T> >(alignment), this->estimator);
				return true;
			}
		};

		/**
		 * Applies the input constraint to a ceres problem
		 * @param problem  ceres problem
		 * @param states list of all states
		 * @param inputs list of all inputs
		 * @param time_diffs list of all time diffs
		 */
		template<typename input_type, typename measurement, int measure_dim>
		static void stateConstrain(ceres::Problem &problem, double *states, double * alignment, int state_size,
				void * data, std::vector<double> &time_diffs, BatchEstimator *estimator) {
			std::vector<input_type>* input_vector = static_cast<std::vector<input_type>*>(data);
			typedef StateDiffFunction<STATE_TYPE_T, measurement, measure_dim> FUNCTION_TYPE;
			for (int i = 0; i < state_size; i++) {
				ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<FUNCTION_TYPE,
				measurement::output_size, MODEL_TYPE::outer_size, ALIGNMENT_SIZE>(
						new FUNCTION_TYPE((*input_vector)[i], estimator));
				problem.AddResidualBlock(cost_function, NULL, &states[i * MODEL_TYPE::outer_size], alignment);
			}
		}

		/**
		 * Applies the input constraint to a ceres problem
		 * @param problem  ceres problem
		 * @param states list of all states
		 * @param inputs list of all inputs
		 * @param time_diffs list of all time diffs
		 */
		template<typename input_type, typename measurement, int measure_dim, int stateNumber>
		static void singleStateConstrain(ceres::Problem &problem, double *states, double * alignment, int state_size,
				void * data, std::vector<double> &time_diffs, BatchEstimator *estimator) {
			std::vector<input_type>* input_vector = static_cast<std::vector<input_type>*>(data);
			typedef StateDiffFunction<STATE_TYPE_T, measurement, measure_dim> FUNCTION_TYPE;

			ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<FUNCTION_TYPE,
			measurement::output_size, MODEL_TYPE::outer_size, ALIGNMENT_SIZE>(
					new FUNCTION_TYPE((*input_vector)[stateNumber], estimator));
			problem.AddResidualBlock(cost_function, NULL, &states[stateNumber * MODEL_TYPE::outer_size], alignment);

		}

		/**
		 * Applies the input constraint to a ceres problem
		 * @param problem  ceres problem
		 * @param states list of all states
		 * @param inputs list of all inputs
		 * @param time_diffs list of all time diffs
		 */
		template<typename measurement, int measure_dim, int stateNumber,
		Eigen::Matrix<double, measure_dim, 1> &measure_target>
		static void singleStateConstrain(ceres::Problem &problem, double *states, double * alignment, int state_size,
				void * data, std::vector<double> &time_diffs, BatchEstimator *estimator) {
			typedef StateDiffFunction<STATE_TYPE_T, measurement, measure_dim> FUNCTION_TYPE;
			ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<FUNCTION_TYPE,
			measurement::output_size, MODEL_TYPE::outer_size, ALIGNMENT_SIZE>(
					new FUNCTION_TYPE(measure_target, estimator));
			problem.AddResidualBlock(cost_function, NULL, &states[stateNumber * MODEL_TYPE::outer_size], alignment);

		}

		/**
		 * Applies the input constraint to a ceres problem
		 * @param problem  ceres problem
		 * @param states list of all states
		 * @param inputs list of all inputs
		 * @param time_diffs list of all time diffs
		 */
		static void inputConstrain(ceres::Problem &problem, double *states, double * alignment, int state_size,
				void * data, std::vector<double> &time_diffs, BatchEstimator *estimator) {
			std::vector<INPUT_TYPE_T<double>>* inputs = static_cast<std::vector<INPUT_TYPE_T<double> >*>(data);
			typedef InputDiffFunction<STATE_TYPE_T, typename MODEL_TYPE::input_measurement, MODEL_TYPE::input_size> FUNCTION_TYPE;
			typedef LowInputFunction<STATE_TYPE_T, typename MODEL_TYPE::input_measurement, MODEL_TYPE::input_size> LOW_FUNCTION_TYPE;
			ceres::LossFunction* loss_function= new ceres::HuberLoss(cfg->lookup("input_huber"));
			ceres::CostFunction* no_input_cost = new ceres::AutoDiffCostFunction<LOW_FUNCTION_TYPE,
			MODEL_TYPE::input_size, MODEL_TYPE::outer_size, ALIGNMENT_SIZE>(
					new LOW_FUNCTION_TYPE(estimator));
			for (int i = 0; i < state_size; i++) {
				if(!estimator->input_mask[i]) {
					problem.AddResidualBlock(no_input_cost, NULL, &states[i * MODEL_TYPE::outer_size], alignment);
				}
				else {
					ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<FUNCTION_TYPE,
					MODEL_TYPE::input_size, MODEL_TYPE::outer_size, ALIGNMENT_SIZE>(
							new FUNCTION_TYPE((*inputs)[i], estimator));
					problem.AddResidualBlock(cost_function, loss_function, &states[i * MODEL_TYPE::outer_size], alignment);
				}
			}
		}

	public:

		/**
		 * Type Erasure for batch constraints
		 */
		struct BatchConstrain {
			virtual void operator()(ceres::Problem &problem, double *states, double * alignment, int state_size,
					void *inputs, std::vector<double> &time_diffs, BatchEstimator *estimator)=0;
		};
		struct BatchConstrainFunctionWrapper: public BatchConstrain {
			void (*function)(ceres::Problem &problem, double *states, double * alignment, int state_size, void *inputs,
					std::vector<double> &time_diffs, BatchEstimator *estimator);
			BatchConstrainFunctionWrapper(
					void (*function)(ceres::Problem &problem, double *states, double * alignment, int state_size,
							void *inputs, std::vector<double> &time_diffs, BatchEstimator *estimator)) :
			function(function) {

			}
			virtual void operator()(ceres::Problem &problem, double *states, double * alignment, int state_size,
					void *inputs, std::vector<double> &time_diffs, BatchEstimator *estimator) {
				function(problem, states, alignment, state_size, inputs, time_diffs, estimator);
			}

		};
		/**
		 * add a constraint function
		 * @param constraint a function which will add a constraint to the ceres problem
		 * it gets access to all states and inputs to add constraints in any  manner
		 */
		void addConstraint(
				void (*constraint)(ceres::Problem &problem, double *states, double * alignment, int state_size, void *,
						std::vector<double> &time_diffs, BatchEstimator *estimator), void * data) {
			constraints.push_back(
					std::make_tuple(
							std::shared_ptr<BatchConstrainFunctionWrapper>(
									new BatchConstrainFunctionWrapper(constraint)), data));
		}
		/**
		 * add a constraint function
		 * @param constraint a function which will add a constraint to the ceres problem
		 * it gets access to all states and inputs to add constraints in any  manner
		 */
		void addConstraint(std::shared_ptr<BatchConstrain> constraint, void * data) {
			constraints.push_back(std::make_tuple(constraint, data));
		}

		/**
		 * Pointer to first state vector
		 * @return states
		 */
		double * getStates() {return states;}

		/**
		 * Number of states
		 * @return num_states
		 */
		size_t getNumStates() {return num_states;}

		/**
		 * Gives the options struct of the solver for editing and reading
		 * @return ceres solver options
		 */
		ceres::Solver::Options * getOptions() {
			return &options;
		}

	protected:
		/**
		 * Add constraints to the minimization problem
		 * @param problem the problem  to constraint
		 * @param states the states on which the constraints can be applied
		 */
		void addConstraints(ceres::Problem &problem, double * states, int state_size, BatchEstimator *estimator) {
			std::cout << "Adding " << constraints.size() << " constraints" << std::endl;
			for (auto constraint : constraints) {
				(*std::get<0>(constraint))(problem, states, alignment, state_size, std::get<1>(constraint), time_diffs,
						estimator);
			}
		}

		//std::shared_ptr<Gnuplot> plot;     //< plot
	public:
		//public so static function can write to them
		std::vector<INPUT_TYPE_T<double> > inputs;//< vector to store all inputs up till now
		std::vector<double> time_diffs;//<vector with time_diffs between measurements
		std::vector<char> input_mask;//!< saves whether the input was valid
	protected:
		std::vector<std::tuple<std::shared_ptr<BatchConstrain>, void *> > constraints;//< vector over constraint functions
		STATE_TYPE_T<double> state_vector;//< the current state vector (from last estimate)
		STATE_TYPE_T<double> lower_bound, upper_bound;// lower and upper bound
		MODEL_TYPE box_model;// model of the estimator
		const bool output_replay;// whether this batch estimator outputs its estimation sequently after it has finished
		volatile double * current_estimation;// the pointer to the data of the last estimation
		volatile int current_replay_index;// the index of the replay
		volatile int current_size;// the size of the current_estimation
		double * states;// pointer to the states
		double * alignment;// the alignment storage
		ALIGNMENT_PARAMETERIZATION align_parameter;
		double skip_time;// the time skipped for this batch
		size_t num_states;//!< number of states in states
		ceres::Solver::Options options;//!< options for the solver
	}
	;

}

#endif /* ESTIMATORS_BATCHESTIMATOR_HPP_ */
