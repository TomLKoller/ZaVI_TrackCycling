/*
 * threaded_objects.cpp
 *
 *  Created on: 26.07.2018
 *      Author: tomlucas
 */

#include "threaded_object.hpp"
#include <glog/logging.h>

namespace zavi {

std::vector<std::thread> ThreadedObject::threads;

/**
 * This function is for internal use to pass to the newly created thread
 *
 * Function to start run in thread
 * @param obj the object whichs run function will be started
 * @param freq the frequency for the run loop
 * @param running whether the thread should still be running
 */
void start(std::shared_ptr<ThreadedObject> obj, double freq, bool (*running)()) {
	LOG(INFO) << "Starting new thread with frequency: "<< freq ;
	obj->run(freq, running);
	LOG(INFO) << "Thread completed";
}

void ThreadedObject::start_thread(std::shared_ptr<ThreadedObject> obj, double freq, bool (*running)()) {
	threads.push_back(std::thread(start, obj, freq, running));
	LOG(INFO) << threads.size() << " threads are now running";
}

void ThreadedObject::joinAll() {
	for (auto thrd = threads.begin(); thrd != threads.end(); thrd++) {
		thrd->join();
	}
	threads.clear();
	LOG(INFO)<<"All threads joined";
}

}
