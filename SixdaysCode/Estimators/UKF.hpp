/*
 * EKF.hpp
 *
 *  Created on: 27.07.2018
 *      Author: tomlucas
 */

#ifndef ESTIMATORS_UKF_HPP_
#define ESTIMATORS_UKF_HPP_

#define FILL_LATER 0

#include "Eigen/Geometry"
#include <Eigen/Cholesky>
#include <OSG_Utils.hpp>

#include "../Plugins/sensor_plugin.hpp"
namespace zavi {
namespace estimator {
/**
 * Base class for all Extended Kalman Filters
 *
 * state_dim Dimension of the State
 * input_dim Dimension of the input Vector
 */
template<typename model>
class UKF: public Estimator {
protected:
	UKF(std::shared_ptr<plugin::SensorPlugin> sensor) :
			box_model(sensor), state_count(0), alignment(alignment.Identity()) {
	}
public:

	typedef model MODEL_TYPE;     //model type
	typedef typename MODEL_TYPE::template OUTER_T<double> STATE_TYPE;
	typedef typename MODEL_TYPE::template INNER_T<double> SIGMA_TYPE;
	typedef Eigen::Matrix<double, MODEL_TYPE::inner_size, MODEL_TYPE::inner_size> STATE_COV_TYPE;     //cov matrix
	typedef Eigen::Matrix<double, MODEL_TYPE::outer_size, MODEL_TYPE::inner_size * 2 + 1> SIGMA_POINTS_TYPE;     //< sigma points matrix type
	typedef Eigen::Matrix<double, MODEL_TYPE::inner_size, MODEL_TYPE::inner_size * 2 + 1> SIGMA_SIGMA_POINTS_TYPE;     //< occurs when the difference of SIGMA_POINTS with the state is drawn

	virtual ~UKF() {
		//smoothAllEstimates();

	}

	STATE_TYPE boxPlus(const STATE_TYPE & state, const SIGMA_TYPE & delta) {
		return box_model.template boxPlus<double>(state, delta);
	}

	SIGMA_TYPE boxMinus(const STATE_TYPE & a, const STATE_TYPE & b) {
		return box_model.template boxMinus<double>(a, b);
	}

	STATE_TYPE stateTransitionFunction(const STATE_TYPE & state, const double time_diff) {
		return box_model.template stateTransitionFunction<double>(state, time_diff);
	}

	/**
	 * Returns the sigma points of the state
	 *
	 *
	 * @param state the state to get the sigma points off
	 * @param cov the covariance of the state
	 * @return a eigen matrix with the sigma points
	 */

	SIGMA_POINTS_TYPE getSigmaPoints(const STATE_TYPE & state, const STATE_COV_TYPE & cov) {
		STATE_COV_TYPE cholesky = cov.llt().matrixL();
		SIGMA_POINTS_TYPE sigma_points = SIGMA_POINTS_TYPE::Zero();
		STATE_COV_TYPE neg_cholesky = STATE_COV_TYPE::Zero() - cholesky;
		Eigen::Matrix<double, MODEL_TYPE::outer_size, MODEL_TYPE::inner_size> negResult, posResult;
		negResult = negResult.Zero();
		posResult = posResult.Zero();
		for (int i = MODEL_TYPE::inner_size - 1; i >= 0; --i) {
			posResult.col(i) = boxPlus(state, cholesky.col(i));
		}
		for (int i = MODEL_TYPE::inner_size - 1; i >= 0; --i) {
			negResult.col(i) = boxPlus(state, neg_cholesky.col(i));
		}
		sigma_points << state, posResult, negResult;

		return sigma_points;
	}

	/**
	 * Simple wrapper to call getSigmaPoints without arguments
	 * @return sigma points of state_vector and covariance
	 */
	virtual SIGMA_POINTS_TYPE getSigmaPoints() {
		return getSigmaPoints(state_vector, covariance);
	}

	/**
	 * Calculates the mean of sigma points
	 * @param sigma_points  the matrix with all sigma points
	 * @param epsilon the stopping criteria
	 * @param max_iterations max iterations of convergence
	 * @return the mean of sigma points
	 */
	STATE_TYPE meanOfSigmaPoints(const SIGMA_POINTS_TYPE &sigma_points, double epsilon = 1e-8,
			int max_iterations = 30) {
		STATE_TYPE mean = sigma_points.col(0);
		STATE_TYPE old_mean = sigma_points.col(0);
		SIGMA_TYPE diff_sum = SIGMA_TYPE::Zero();
		int iterations = 0;
		do {
			iterations++;
			old_mean = mean;
			diff_sum = diff_sum.Zero();
			for (int i = MODEL_TYPE::inner_size * 2; i >= 0; --i) {
				diff_sum += boxMinus(mean, sigma_points.col(i));
			}
			diff_sum /= 2. * MODEL_TYPE::inner_size + 1.;
			mean = boxPlus(mean, diff_sum);
		} while (iterations <= max_iterations && boxMinus(mean, old_mean).norm() > epsilon);
		if (iterations > max_iterations)
			printf("Warning: stopped due to excess of iterations");
		return mean;
	}

	/**
	 * Calculates the mean of sigma points
	 * @param sigma_points  the matrix with all sigma points
	 * @param epsilon the stopping criteria
	 * @param max_iterations max iterations of convergence
	 * @return the mean of sigma points
	 */
	template<int measure_dim, int measure_inner_dim>
	Eigen::Matrix<double, measure_dim, 1> meanOfSigmaPoints(
			const Eigen::Matrix<double, measure_dim, MODEL_TYPE::inner_size * 2 + 1> &sigma_points,
			Eigen::Matrix<double, measure_dim, 1> (*boxplus_m)(const Eigen::Matrix<double, measure_dim, 1> & state,
					const Eigen::Matrix<double, measure_inner_dim, 1> & delta),
			Eigen::Matrix<double, measure_inner_dim, 1> (*boxminus_m)(const Eigen::Matrix<double, measure_dim, 1> & a,
					const Eigen::Matrix<double, measure_dim, 1> & b), double epsilon = 1e-6, int max_iterations = 30) {

		Eigen::Matrix<double, measure_dim, 1> mean = sigma_points.col(0);
		Eigen::Matrix<double, measure_dim, 1> old_mean = sigma_points.col(0);
		Eigen::Matrix<double, measure_inner_dim, 1> diff_sum = Eigen::Matrix<double, measure_inner_dim, 1>::Zero();
		int iterations = 0;
		do {
			iterations++;
			old_mean = mean;
			diff_sum = diff_sum.Zero();
			for (int i = MODEL_TYPE::inner_size * 2; i >= 0; --i) {
				diff_sum += boxminus_m(mean, sigma_points.col(i));
			}
			diff_sum /= 2. * MODEL_TYPE::inner_size + 1.;
			mean = boxplus_m(mean, diff_sum);
		} while (iterations <= max_iterations && boxminus_m(mean, old_mean).norm() > epsilon);
		if (iterations > max_iterations)
			printf("Warning: stopped due to excess of iterations");
		return mean;
	}
	/**
	 * Saves all releveant states for smoothing
	 * @param input the input u
	 * @param time_diff time since last call
	 */
	void saveStatesForSmoothing(double time_diff) {
		state_count++;
		past_states.push_back(state_vector);
		past_states_smoothed.push_back(state_vector);
		past_covs.push_back(covariance);
		past_timediffs.push_back(time_diff);
	}
	/**
	 * Does  dynamic step in EKF
	 *
	 * @param input the input u
	 * @param time_diff time since last call
	 */
	void dynamicStep(double time_diff) {
		SIGMA_POINTS_TYPE sigma_points = getSigmaPoints();
		for (int i = MODEL_TYPE::inner_size * 2; i >= 0; --i) {
			sigma_points.col(i) = stateTransitionFunction(sigma_points.col(i), time_diff);
		}
		state_vector = meanOfSigmaPoints(sigma_points);
		//printf(state_vector);
		//printf(" ");
		SIGMA_SIGMA_POINTS_TYPE result = SIGMA_SIGMA_POINTS_TYPE::Zero();
		for (int i = MODEL_TYPE::inner_size * 2; i >= 0; --i) {
			result.col(i) = boxMinus(state_vector, sigma_points.col(i));
		}
		covariance = 0.5 * (result * result.transpose()) + processNoise(time_diff);
		//printf("dynamic");
		//printf(state_vector.block(3, 0, 3, 1).norm());
	}
	/**
	 * Does  a  measurement update in UKF
	 * @param measurement the measurement
	 * @param time_diff time since last call
	 * @param measure_function function to map a state to a predicted measurement
	 * @param noise_function gives the measurement noise with time_diff
	 * @param boxplus_m boxplus for measurement
	 * @param boxminus_m boxminus for measurement
	 * measure dim is the dimension of the measurement vector
	 */
	template<int measure_inner_dim, typename functor, int measure_dim>
	void measurementStepManifold(const Eigen::Matrix<double, measure_dim, 1> & measurement, double time_diff,
			const functor & measure_function, const Eigen::Matrix<double, measure_inner_dim, measure_inner_dim> & noise,
			Eigen::Matrix<double, measure_dim, 1> (*boxplus_m)(const Eigen::Matrix<double, measure_dim, 1> & state,
					const Eigen::Matrix<double, measure_inner_dim, 1> & delta),
			Eigen::Matrix<double, measure_inner_dim, 1> (*boxminus_m)(const Eigen::Matrix<double, measure_dim, 1> & a,
					const Eigen::Matrix<double, measure_dim, 1> & b), void *prior = NULL) {
		SIGMA_POINTS_TYPE sigma_points = getSigmaPoints();
		Eigen::Matrix<double, measure_dim, MODEL_TYPE::inner_size * 2 + 1> expected_zs;
		for (int i = MODEL_TYPE::inner_size * 2; i >= 0; --i) {
			expected_zs.col(i) = measure_function(STATE_TYPE(sigma_points.col(i)), alignment, prior);
		}
		Eigen::Matrix<double, measure_dim, 1> mean_z = meanOfSigmaPoints(expected_zs, boxplus_m, boxminus_m);     //< expected measurement mean
		Eigen::Matrix<double, measure_inner_dim, MODEL_TYPE::inner_size * 2 + 1> diff_z;
		for (int i = MODEL_TYPE::inner_size * 2; i >= 0; --i) {
			diff_z.col(i) = boxminus_m(mean_z, expected_zs.col(i));
		}
		Eigen::Matrix<double, measure_inner_dim, measure_inner_dim> sigma_z = 0.5 * (diff_z * diff_z.transpose())
				+ noise;
		SIGMA_SIGMA_POINTS_TYPE result = SIGMA_SIGMA_POINTS_TYPE::Zero();
		for (int i = MODEL_TYPE::inner_size * 2; i >= 0; --i) {
			result.col(i) = boxMinus(state_vector, sigma_points.col(i));
		}
		Eigen::Matrix<double, MODEL_TYPE::inner_size, measure_inner_dim> sigma_xz = 0.5 * (result * diff_z.transpose());
		Eigen::Matrix<double, MODEL_TYPE::inner_size, measure_inner_dim> kalman_gain = sigma_xz * sigma_z.inverse();
		Eigen::Matrix<double, MODEL_TYPE::inner_size, 1> delta = kalman_gain * boxminus_m(mean_z, measurement);
		STATE_COV_TYPE sigma_t = covariance - (kalman_gain * sigma_z * kalman_gain.transpose());
		SIGMA_POINTS_TYPE sigma_points_second = SIGMA_POINTS_TYPE::Zero();
		STATE_COV_TYPE cholesky = sigma_t.llt().matrixL();
		Eigen::Matrix<double, MODEL_TYPE::outer_size, MODEL_TYPE::inner_size> negResult, posResult;
		negResult = negResult.Zero();
		posResult = posResult.Zero();
		STATE_COV_TYPE neg_cholesky = STATE_COV_TYPE::Zero() - cholesky;
		for (int i = MODEL_TYPE::inner_size - 1; i >= 0; --i) {
			posResult.col(i) = boxPlus(state_vector,
					box_model.template boxPlusInnerSpace<double>(delta, cholesky.col(i)));
		}
		for (int i = MODEL_TYPE::inner_size - 1; i >= 0; --i) {
			negResult.col(i) = boxPlus(state_vector,
					box_model.template boxPlusInnerSpace<double>(delta, neg_cholesky.col(i)));
		}
		sigma_points_second << boxPlus(state_vector, delta), posResult, negResult;

		state_vector = meanOfSigmaPoints(sigma_points_second);
		for (int i = MODEL_TYPE::inner_size * 2; i >= 0; --i) {
			result.col(i) = boxMinus(state_vector, sigma_points_second.col(i));
		}
		covariance = 0.5 * (result * result.transpose());

	}
	/**
	 * Does  a  measurement update in UKF
	 * @param measurement the measurement
	 * @param time_diff time since last call
	 * @param measure_function function to map a state to a predicted measurement
	 * @param noise_function gives the measurement noise with time_diff
	 * measure dim is the dimension of the measurement vector
	 */
	template<typename functor, int measure_dim>
	void measurementStep(const Eigen::Matrix<double, measure_dim, 1> & measurement, double time_diff,
			const functor & measure_function, const Eigen::Matrix<double, measure_dim, measure_dim> & noise,
			void * prior = NULL) {

		SIGMA_POINTS_TYPE sigma_points = getSigmaPoints();
		Eigen::Matrix<double, measure_dim, MODEL_TYPE::inner_size * 2 + 1> expected_zs;
		for (int i = MODEL_TYPE::inner_size * 2; i >= 0; --i) {
			expected_zs.col(i) = measure_function(STATE_TYPE(sigma_points.col(i)), alignment, prior);
		}
		Eigen::Matrix<double, measure_dim, 1> mean_z = expected_zs.rowwise().mean();     //< expected measurement mean
		Eigen::Matrix<double, measure_dim, MODEL_TYPE::inner_size * 2 + 1> diff_z = expected_zs.colwise() - mean_z;
		Eigen::Matrix<double, measure_dim, measure_dim> sigma_z = 0.5 * (diff_z * diff_z.transpose()) + noise;

		SIGMA_SIGMA_POINTS_TYPE result = SIGMA_SIGMA_POINTS_TYPE::Zero();
		for (int i = MODEL_TYPE::inner_size * 2; i >= 0; --i) {
			result.col(i) = boxMinus(state_vector, sigma_points.col(i));
		}
		Eigen::Matrix<double, MODEL_TYPE::inner_size, measure_dim> sigma_xz = 0.5 * (result * diff_z.transpose());

		Eigen::Matrix<double, MODEL_TYPE::inner_size, measure_dim> kalman_gain = sigma_xz * sigma_z.inverse();

		state_vector = boxPlus(state_vector, kalman_gain * (measurement - mean_z));

		covariance = covariance - kalman_gain * sigma_xz.transpose();
	}
	/**
	 * wrapper to call measurementstep with different arguments
	 */
	template<int measure_dim>
	struct MeasurementWrapper {
		Eigen::Matrix<double, measure_dim, 1> (*function)(const STATE_TYPE &, void *);
		MeasurementWrapper(Eigen::Matrix<double, measure_dim, 1> (*function)(const STATE_TYPE &, void *)) :
				function(function) {

		}
		Eigen::Matrix<double, measure_dim, 1> operator()(const STATE_TYPE & state,
				const Eigen::Matrix<double, 4, 4> & alignment, void *prior) const {
			return function(state, prior);
		}

	};
	template<int measure_dim>
	void measurementStep(const Eigen::Matrix<double, measure_dim, 1> & measurement, double time_diff,
			Eigen::Matrix<double, measure_dim, 1> (*measure_function)(const STATE_TYPE &, void * prior),
			const Eigen::Matrix<double, measure_dim, measure_dim> & noise, void * prior = NULL) {
		measurementStep(measurement, time_diff, MeasurementWrapper<measure_dim>(measure_function), noise, prior);
	}

	/**
	 * Smooth previous estimates with new knowledge
	 *
	 * This is taken from:
	 *
	 * Unscented Rauch–Tung–Striebel Smoother by Simo S"arkk"a
	 *
	 * @param k_length the amount of steps to smooth back
	 * @param k_start the starting index for the smoothing
	 */
	void smoothEstimates(unsigned int k_length, unsigned int k_start) {
		if (k_start > state_count - 1) {
			LOG(ERROR)<<"Trying to smooth from a non existent state";
			return;
		}

		if (k_length > k_start) {
			LOG(ERROR)<< "Trying to smooth more states than are before k_start";
			return;
		}
		for (unsigned int k = k_start; k > k_start - k_length; k--) {
			SIGMA_POINTS_TYPE sigma_points = getSigmaPoints(past_states_smoothed[k], past_covs[k]);
			SIGMA_POINTS_TYPE sigma_points_plus;
			for (int i = 0; i < MODEL_TYPE::inner_size * 2 + 1; i++) {
				sigma_points_plus.col(i) = stateTransitionFunction(sigma_points.col(i), past_timediffs[k]);
			}

			STATE_TYPE mean = meanOfSigmaPoints(sigma_points_plus);
			SIGMA_SIGMA_POINTS_TYPE result_plus;
			for (int i = 0; i < MODEL_TYPE::inner_size * 2 + 1; i++) {
				result_plus.col(i) = boxMinus(mean, sigma_points_plus.col(i));
			}

			SIGMA_SIGMA_POINTS_TYPE result;
			STATE_COV_TYPE cov = 0.5*(result_plus * result_plus.transpose())+ processNoise(past_timediffs[k]);
			for (int i = 0; i < MODEL_TYPE::inner_size * 2 + 1; i++) {
				result.col(i) = boxMinus(past_states_smoothed[k], sigma_points.col(i));
			}
			STATE_COV_TYPE c_k_plus =0.5* result * result_plus.transpose();
			STATE_COV_TYPE d_k = c_k_plus * cov.inverse();
			//past_states_smoothed[k] = boxPlus(past_states_smoothed[k],
			//		d_k * boxMinus(mean, past_states_smoothed[k + 1]));
			//past_covs[k] = past_covs[k] + d_k * (past_covs[k + 1] - cov) * d_k.transpose();

			//from here the second sigma propagation applies
			SIGMA_TYPE delta=d_k * boxMinus(mean, past_states_smoothed[k + 1]);
			STATE_COV_TYPE pst_cov_k=past_covs[k] + d_k * (past_covs[k + 1] - cov) * d_k.transpose();

			SIGMA_POINTS_TYPE sigma_points_second = SIGMA_POINTS_TYPE::Zero();
			STATE_COV_TYPE cholesky = pst_cov_k.llt().matrixL();
			Eigen::Matrix<double, MODEL_TYPE::outer_size, MODEL_TYPE::inner_size> negResult, posResult;
			negResult = negResult.Zero();
			posResult = posResult.Zero();
			STATE_COV_TYPE neg_cholesky = STATE_COV_TYPE::Zero() - cholesky;
			for (int i = MODEL_TYPE::inner_size - 1; i >= 0; --i) {
				posResult.col(i) = boxPlus(past_states_smoothed[k],
						box_model.template boxPlusInnerSpace<double>(delta, cholesky.col(i)));
			}
			for (int i = MODEL_TYPE::inner_size - 1; i >= 0; --i) {
				negResult.col(i) = boxPlus(past_states_smoothed[k],
						box_model.template boxPlusInnerSpace<double>(delta, neg_cholesky.col(i)));
			}
			sigma_points_second << boxPlus(past_states_smoothed[k], delta), posResult, negResult;

			past_states_smoothed[k] = meanOfSigmaPoints(sigma_points_second);
			for (int i = MODEL_TYPE::inner_size * 2; i >= 0; --i) {
				result.col(i) = boxMinus(past_states_smoothed[k], sigma_points_second.col(i));
			}
			past_covs[k] = 0.5 * (result * result.transpose());
		}
	}
	/**
	 * Perform smoothing from the newest estimate
	 * @param k_length the amount of estimates to smooth
	 */
	void smoothEstimates(unsigned int k_length) {

		smoothEstimates(k_length, state_count - 2);

	}
	/**
	 * Perform smoothing from the newest estimate for all estimates
	 */
	void smoothAllEstimates() {
		smoothEstimates(state_count - 2);
	}

	static void smoothCallback(plugin::SensorPlugin * plug, void* estimator, double time) {
		UKF * esti = static_cast<UKF *>(estimator);
		esti->smoothAllEstimates();
	}

	/**
	 * Sets the start estimat \hat(x) (0)
	 * @param start_state the starting state vector
	 * @param start_cov  the starting covariance
	 */
	virtual inline void setStart(const STATE_TYPE & start_state, const STATE_COV_TYPE &start_cov) {
		state_vector = start_state;
		covariance = start_cov;
	}

	/**
	 * Gives the dimension of the state
	 * @return Dimension of the state vector
	 */
	static constexpr int getStateDim() {
		return MODEL_TYPE::outer_size;
	}

	virtual inline STATE_COV_TYPE processNoise(const double time_diff) {
		STATE_COV_TYPE matrix = box_model.getStateSTD(time_diff).asDiagonal();

		//zavi::printf(matrix);
		return matrix;
	}

	inline MODEL_TYPE getBoxModel() {
		return box_model;
	}
	inline STATE_TYPE getStateVector() {
		return state_vector;
	}
	inline STATE_COV_TYPE getCov() {
		return covariance;
	}

	std::vector<STATE_TYPE> & getSmoothedStates() {
		return past_states_smoothed;
	}

	void setAlignment(const Eigen::Matrix4d & alignment) {
		this->alignment=alignment;
	}

protected:
	MODEL_TYPE box_model;     // The Model type
	STATE_TYPE state_vector;//< the current estimated state x
	STATE_COV_TYPE covariance;//< the estimated cov(x)
	std::vector<STATE_TYPE> past_states;//< all past states
	std::vector<STATE_TYPE> past_states_smoothed;//< all past states
	//std::vector<INPUT_TYPE> past_inputs;//< all past inputs
	std::vector<double> past_timediffs;//< all past time_diffs

	std::vector<STATE_COV_TYPE> past_covs;// < all past covariance matrices
	unsigned int state_count;//< the current state index
	Eigen::Matrix<double, 4, 4> alignment;
};

}
/* namespace estimator */
}
/* namespace zavi */

#endif /* ESTIMATORS_UKF_HPP_ */
