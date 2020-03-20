/*
 * Callbacker.hpp
 *
 *  Created on: 07.02.2019
 *      Author: tomlucas
 */

#ifndef CALLBACKER_HPP_
#define CALLBACKER_HPP_

#include <vector>
#include <glog/logging.h>
namespace zavi{

/**
 * Base class for callbacks
 */
class Callback{
public:
	/**
	 * The called callback
	 */
	virtual void operator() (void *)=0;
	virtual ~Callback(){
		DLOG(INFO)<<"Callback got destroyed";
	}
};
/**
 * Callbackers can notify callbacks whenever certain tasks are completed so they can derive data from the callbacker
 */
class Callbacker{
public:
	virtual ~Callbacker(){

	}
	/**
	 * Add a callback to call on notifyCallbacks(index)
	 * @param callback the callback to call
	 * @param index the index when to call (subclassses have to define which index is called when)
	 */
	void addCallback(std::shared_ptr<Callback> callback, int index){
		callbacks.push_back(callback);
		call_indices.push_back(index);
	}
protected:
	/**
	 * call all callbacks registered with given index
	 * @param index
	 */
	void notifyCallbacks(int index){
		for(unsigned int i=0; i < callbacks.size();i++){
			if(call_indices[i]==index){
				callbacks[i]->operator ()(this);
			}
		}
	}



private:
	std::vector<std::shared_ptr<Callback>> callbacks; //!< all registered callbacks
	std::vector<int> call_indices;  //!< there assignment numbers
};

}


#endif /* CALLBACKER_HPP_ */
