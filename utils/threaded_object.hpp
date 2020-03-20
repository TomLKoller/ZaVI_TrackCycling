/*
 * threaded_object.hpp
 *
 *  Created on: 25.07.2018
 *      Author: tomlucas
 */

#ifndef THREADED_OBJECT_HPP_
#define THREADED_OBJECT_HPP_

#include <thread>
#include <vector>
namespace zavi {

/**
 * Base Class for several objects which run in their own thread
 */
class ThreadedObject {
public:
	virtual ~ThreadedObject() {

	}
	/**
	 * Runs the thread task with given frequency
	 *
	 * @param freq Frequency of the loop, can be ignored by subclass, overwrite function to do so
	 * @param running bool pointer to determine whether this object can still be alive
	 */
	virtual void run(double freq, bool (*running)())=0;

	/**
	 * Starts the run function in  thread
	 * @param the object whichs runs function will be started
	 * @param freq the frequency for the run loop
	 * @param running whether the thread should be still running
	 */
	static void start_thread(std::shared_ptr<ThreadedObject> obj,double freq, bool (*running)()) ;
	/**
	 * JoinAllThreads
	 */
	static void joinAll() ;
private:
	static std::vector<std::thread> threads;     //!< a list over all started threads
};

}     //zavi

#endif /* THREADED_OBJECT_HPP_ */
