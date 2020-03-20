/*
 * StateReaderCallback.hpp
 *
 *  Created on: 13.02.2019
 *      Author: tomlucas
 */

#ifndef ESTIMATORS_CALLBACKS_UKFSTATESAVERCALLBACK_HPP_
#define ESTIMATORS_CALLBACKS_UKFSTATESAVERCALLBACK_HPP_
#include "../UKF.hpp"
#include <zavi/utils/Callbacker.hpp>
#include <vector>
namespace zavi::estimator::callbacks{

	/**
	 * Callback saves all states of an ukf estimator
	 */
	template<typename ukf_estimator>
	class UKFStateSaverCallback: public Callback{
public:
		std::vector<typename ukf_estimator::STATE_TYPE > states; //!< the saved states
		virtual ~UKFStateSaverCallback(){}
		virtual void operator()(void * callbacker){
			ukf_estimator * estimator=static_cast<ukf_estimator *>(callbacker);
			states.push_back(estimator->getStateVector());
		}

	};

}


#endif /* ESTIMATORS_CALLBACKS_UKFSTATESAVERCALLBACK_HPP_ */
