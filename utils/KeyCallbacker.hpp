/*
 * KeyCallbacker.hpp
 *
 *  Created on: 20.03.2019
 *      Author: tomlucas
 */

#ifndef KEYCALLBACKER_HPP_
#define KEYCALLBACKER_HPP_

#include "Callbacker.hpp"
#include <ZaVI_Utils.hpp>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>

namespace zavi {
/**
 * Double purpose class
 *
 * assign callbacks with addCallback
 *
 * use start thread to call callback frequently
 *
 *or create class with a key to only apply callback when key is clicked
 */
class KeyCallbacker: public Callbacker {
private:
	class KeyReader: public osgGA::GUIEventHandler {
	public:
		/**
		 *
		 * creates a pseudo plugin and registers it to the viewer
		 * @param key on which key callbacks are activated
		 * @param viewer the viewer object
		 */
		KeyReader(osgGA::GUIEventAdapter::KeySymbol key, std::shared_ptr<KeyCallbacker> callbacker) :
				callbacker(callbacker), key(key) {
		}

		std::shared_ptr<KeyCallbacker> callbacker;
		osgGA::GUIEventAdapter::KeySymbol key;     //< the key to react on
		/**
		 * Function to handle click events
		 * @param ea the gui event adapter
		 * @param aa an action adapter
		 * @return
		 */
		virtual bool handle(const osgGA::GUIEventAdapter & ea, osgGA::GUIActionAdapter &aa) {
			static bool isUp = true;
			if (ea.getEventType() == osgGA::GUIEventAdapter::EventType::KEYDOWN && ea.getKey() == key) {
				isUp = false;
				LOG(INFO)<< "Registered Key Event for key " << key;
				return true;
			}
			if (ea.getEventType() == osgGA::GUIEventAdapter::EventType::KEYUP && ea.getKey() == key && !isUp) {
				isUp = true;
				callbacker->notifyCallbacks(CI_KEY);
				LOG(INFO)<< "Registered Key Event for key " << key << " and notified callbacks";
				return true;
			}
			return false;
		}

	};

protected:

public:

	static std::shared_ptr<KeyCallbacker> createKeyCallbacker(osgViewer::Viewer *viewer = NULL,
			osgGA::GUIEventAdapter::KeySymbol key = osgGA::GUIEventAdapter::KEY_Escape) {
		std::shared_ptr<KeyCallbacker> callbacker(new KeyCallbacker());
		if (viewer != NULL) {
			viewer->addEventHandler(osg::ref_ptr<KeyReader>(new KeyReader(key, callbacker)));
			LOG(INFO)<< "Registered new KeyReader with key " << key;
		}
		return callbacker;
	}

	enum CallbackIndices {
		CI_LOOP, CI_KEY
	};

	void run(double freq, bool (*running)()) {
		Timer loop(freq);
		//create noises
		while (running()) {
			notifyCallbacks(CI_LOOP);
			loop.wait();
		}

	}

private:
};
}
//zavi::plugin

#endif /* KEYCALLBACKER_HPP_ */
