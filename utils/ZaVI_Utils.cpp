/*
 * ZaVI_Utils.cpp
 *
 *  Created on: 24.07.2018
 *      Author: tomlucas
 */

#include "ZaVI_Utils.hpp"
namespace zavi {

double Timer::time_lapse;
bool Timer::file_mode = false;     // whether the timer takes system time or file time
volatile double Timer::file_time = 0;     // only relevant in file mode, the current time dictated by file
bool Timer::replay_mode = false;     // whether the visual time takes system time or dictated time
volatile double Timer::replay_time = 0;     // only relevant in replay mode, the current time dictated by repla
std::chrono::system_clock::time_point Timer::time_start;

Timer::Timer(double freq) :
		diff(TIME_BASE(std::lround(1/freq/time_lapse/BASE_TO_SEC))) {
			reset();
		}


void Timer::wait() {
	auto now = std::chrono::system_clock::now();
	time += diff;     // implicit time reset
	if (TIME_BASE(time-now).count() < 0) {
		LOG(WARNING)<< " Can not keep up with runtime. Estimated Frequency is " <<1/((TIME_BASE(now-last_time).count()*time_lapse*BASE_TO_SEC)) << "Hz.";
		reset();     // hard reset
	}
	else {
		std::this_thread::sleep_until(time);
	}
	last_time = now;
}

void Timer::reset() {
	time = std::chrono::system_clock::now();
	last_time = time;
}


void Timer::initTime(double time_lapse) {
	time_start = std::chrono::system_clock::now();
	Timer::time_lapse = time_lapse;
}


double Timer::getSeconds() {
	return diff.count() * BASE_TO_SEC;
}

double Timer::getNow() {
	if (!file_mode) {
		auto timepoint = std::chrono::system_clock::now() - time_start;
		auto now = std::chrono::duration_cast<TIME_BASE>(timepoint);
		return now.count() * time_lapse * BASE_TO_SEC;
	} else {
		return file_time;
	}
}

double Timer::getReplayTime() {
	if (!replay_mode)
		return getNow();
	else
		return replay_time;
}
/**
 * Toggles the file Mode of all timers
 * time has to be set manually by calling setFileTime
 * @see setFileTime
 * @param mode default file_time=true , system_time=false
 */
void Timer::activateFileMode(bool mode ) {
	LOG(INFO)<<( mode ? "Activated" : "Disabled" ) <<" file time";
	file_mode=mode;
}


void Timer::setFileTime(double file_time) {
	Timer::file_time = file_time;
}


void Timer::activateReplay(bool mode ) {
	LOG(INFO)<<( mode ? "Activated" : "Disabled" ) <<" replay";
	replay_mode=mode;
}


void Timer::setReplayTime(double replay_time) {
	Timer::replay_time = replay_time;
}

}     // zavi
