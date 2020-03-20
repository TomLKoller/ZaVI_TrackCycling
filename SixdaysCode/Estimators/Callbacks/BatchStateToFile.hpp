/*
 * BatchStateToFile.hpp
 *
 *  Created on: 19.03.2019
 *      Author: tomlucas
 */

#ifndef ESTIMATORS_CALLBACKS_BATCHSTATETOFILE_HPP_
#define ESTIMATORS_CALLBACKS_BATCHSTATETOFILE_HPP_

#include <iostream>
#include <fstream>
#include <Callbacker.hpp>
#include "../BatchEstimator.hpp"
namespace zavi
::estimator::callbacks {
	template<typename BATCH_TYPE>
	/**
	 * This callback is used to store the results of a batch estimator in files for later reload
	 */
	class BatchStateToFileCallback: public Callback {
	private:
		const char *filename; //the filename to store the states
		const char *time_file; //!< the fileaname to store the time_diffs
		const char *input_mask; //!< the filename to store the input_mask
	public:

		/**
		 *
		 * @param filename  batch state file
		 * @param time_file  time diffs file
		 * @param input_mask  input mask file
		 */
		BatchStateToFileCallback(const char * filename, const char * time_file, const char * input_mask):filename(filename),time_file(time_file),input_mask(input_mask){

		}
		virtual ~BatchStateToFileCallback() {
		}
		/**
		 * Just passes the callback operator for sensors to the real callback handler
		 * @param the calling sensor
		 * @param estimator the registered estimator
		 * @param time the current time
		 */
		virtual void operator()(plugin::SensorPlugin *plugin, void * estimator, double time) {
			this->operator()(estimator);
		}
		/**
		 * callback which saves all data of a batch estimator in binary format
		 * @param callbacker a batch estimator
		 */
		virtual void operator()(void * callbacker) {
			BATCH_TYPE * estimator = static_cast<BATCH_TYPE *>(callbacker);
			storeData(filename,estimator->getStates(), estimator->getNumStates()* BATCH_TYPE::MODEL_TYPE::outer_size*sizeof(double));
			storeData(time_file,&(estimator->time_diffs.front()), estimator->time_diffs.size()*sizeof(double));
			storeData(input_mask, &(estimator->input_mask.front()), estimator->input_mask.size()*sizeof(char));

		}
		/**
		 * Stores data in a file
		 * @param filename the target file
		 * @param data the data pointer
		 * @param size the size of the data
		 * @return 0 on success 1 on failure
		 */
		static int storeData(const char * filename, void * data, size_t size){
			std::ofstream out(filename, std::ios::out | std::ios::binary);
			if(!out) {
			   	LOG(ERROR) << "Unable to store data in file: " << filename;
			    return 1;
			   }
			out.write((char*) data, size);
			out.close();
			return 0;

		}

	};

}

#endif /* ESTIMATORS_CALLBACKS_BATCHSTATETOFILE_HPP_ */
