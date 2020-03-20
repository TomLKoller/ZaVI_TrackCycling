/*@file
 * This File includes all parts of the ZaVI utility library
 * ZaVI_Utils.hpp
 *
 *  Created on: 16.07.2018
 *      Author: Tom Koller
 */

#ifndef ZAVI_UTILS_HPP_
#define ZAVI_UTILS_HPP_

#include <iostream>
#include <chrono>
#include <thread>
#include <math.h>
#include <glog/logging.h>
#include <cassert>
#include <Eigen/Core>
#define TIME_BASE std::chrono::nanoseconds
#define BASE_TO_SEC 1e-9

/**
 * Determines whether all matrix elements are not inf or nan
 * @param matrix The matrix to check
 * @return	whether all values are finite
 */
template<typename Derived>
bool isfinite(const Eigen::MatrixBase<Derived> & matrix) {
	return matrix.allFinite();
}
/**
 * does nothing just for variadic resolve of assert_inputs(T first,ARGS)
 */
void inline assert_inputs() {

}
/**
 * asserts whether all input numbers have finite values
 *
 * Uses variadic Evaluation to check an undefined amount of variables
 * @param first  the argument which is evaluated at this call
 * @param args   arguments for next calls
 */
template<typename T, typename ... Args>
void assert_inputs(T first, Args ... args) {
	assert(isfinite(first));
	assert_inputs(args ...);
}

/**
 * Holds all functions of ZaVI project
 */
namespace zavi {
/**
 * print an object on std::cout
 *
 * default behavior is to print an '\n' after the object can be changed via parameter end
 *
 * @param msg the object to print, needs to implement stream operation
 * @param end and ending string defaults to '\n'
 */
template<class T> void printf(T msg, const char * end = "\n") __attribute__((no_instrument_function));

template<class T> void printf(T msg, const char * end) {
	std::cout << msg << end << std::endl;
}

/**
 * Class to implement loops with given frequency
 *
 * also holds overall time_lapse functions
 *
 * The static functions can be used to get the current program time there are 3 mechanics:
 *
 *
 * Normal mode: just gives the time since the program started
 * Filemode: Programm reads values from a file with time_stamps so all times are setted to this
 * Replay-Mode: Enables the use of a replay, also simultaneosly with a visualisation of current values ( usefull for batch estimator which is not real time capable)
 *
 */
class Timer {
public:
	/**
	 * Create timer, reset time
	 * @param freq the frequency of the loop
	 */
	Timer(double freq);

	/**
	 * wait until time + 1/freq has passed
	 *
	 * Call this at the end of the loop to run it in the desired frequency
	 *
	 * Resets time
	 */
	void wait();

	/**
	 * Reset time if needed for any reason
	 *
	 * The timer acts as the current interval has just begun
	 */
	void reset();
	/**
	 * Set the timelapse of the whole program
	 *
	 * has to be called before any timer is constructed
	 * @param time_lapse
	 */
	static void initTime(double time_lapse);

	/**
	 * Gives the interval of the timer in seconds
	 * @return intervall till next loop in seconds
	 */
	double getSeconds();

	/**
	 * Get the current time  with time_lapse
	 *
	 * This function returns the system time since the start
	 * or the current file time if in file mode
	 *
	 * @return The passed time from start/time_lapse
	 */
	static double getNow();
	/**
	 * returns the replay time
	 * @return replay_time if in replay mode else getNow()
	 * @see getNow()
	 */
	static double getReplayTime();
	/**
	 * Toggles the file Mode of all timers
	 * time has to be set manually by calling setFileTime
	 * @see setFileTime
	 * @param mode default file_time=true , system_time=false
	 */
	static void activateFileMode(bool mode = true);
	/**
	 * Sets the current file_time
	 * @param file_time the current time of the file
	 */
	static void setFileTime(double file_time);

	/**
	 * Toggles the replay Mode of all timers
	 * time has to be set manually by calling setReplayTime
	 * @see setReplayTime
	 * @param mode default replay_time=true , system_time=false
	 */
	static void activateReplay(bool mode = true);
	/**
	 * Sets the current file_time
	 * @param replay_time the current time of the file
	 */
	static void setReplayTime(double replay_time);

	static double time_lapse;     //!< Time Lapse of the whole program

private:
	static bool file_mode;     //!< whether the timer takes system time or file time
	static volatile double file_time;     //!< only relevant in file mode, the current time dictated by file
	static bool replay_mode;     //!< whether the visual time takes system time or dictated time
	static volatile double replay_time;     //!< only relevant in replay mode, the current time dictated by replayer
	static std::chrono::system_clock::time_point time_start;     //!< starting time of the program
	std::chrono::system_clock::time_point last_time;     //!< the last time wait was called
	std::chrono::system_clock::time_point time;    //!< the reference timepoint for waiting
	TIME_BASE diff;     //!< the amount of TIME_BASE between each timepoint to reach freq
};

}
//zavi

#endif /* ZAVI_UTILS_HPP_ */
