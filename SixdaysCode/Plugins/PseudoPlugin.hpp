/*
 * PseudoSensor.hpp
 *
 *  Created on: 09.08.2018
 *      Author: tomlucas
 */

#ifndef PLUGINS_PSEUDOPLUGIN_HPP_
#define PLUGINS_PSEUDOPLUGIN_HPP_

#include "sensor_plugin.hpp"
#include <zavi/utils/ZaVI_Utils.hpp>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>

namespace zavi
::plugin {
	/**
	 * Double purpose class
	 *
	 * assign callbacks with addDataCallback
	 *
	 * use start thread to call callback frequently
	 *
	 *or create class with a key to only apply callback when key is clicked
	 */
	class PseudoPlugin : public SensorPlugin, public osgGA::GUIEventHandler {
	public:
		/**
		 * Do not use if you plan to call start_thread on this
		 *
		 * creates a pseudo plugin and registers it to the viewer
		 * @param key on which key callbacks are activated
		 * @param viewer the viewer object
		 */
		PseudoPlugin(osgGA::GUIEventAdapter::KeySymbol key,osgViewer::Viewer *viewer):key(key),asEventHandler(true) {
			viewer->addEventHandler(this);
		}
		/**
		 * Empty Constructor if start_thread is desired for use
		 */
		PseudoPlugin():key(osgGA::GUIEventAdapter::KeySymbol::KEY_Escape),asEventHandler(false) {

		}

		void run(double freq, bool (* running)()) {
			if(asEventHandler) {     // return if started as eventhandler
				LOG(WARNING) <<"Tried to start an event handler as thread";
				return;
			}
			Timer loop(freq);
			//create noises
			while(running()) {
				newDataAvailable();
				loop.wait();
			}

		}
		/**
		 * Function to handle click events
		 * @param ea the gui event adapter
		 * @param aa an action adapter
		 * @return
		 */
		virtual bool handle(const osgGA::GUIEventAdapter & ea, osgGA::GUIActionAdapter &aa) {
			static bool isUp=true;
			if(ea.getEventType()!=osgGA::GUIEventAdapter::EventType::KEYDOWN &&ea.getKey()==key) {
				isUp=false;
				return true;
			}
			if(ea.getEventType()!=osgGA::GUIEventAdapter::EventType::KEYUP &&ea.getKey()==key && !isUp ) {
				isUp=true;
				newDataAvailable();
				return true;
			}
			return false;
		}
	private:
		osgGA::GUIEventAdapter::KeySymbol key;     //< the key to react on
		bool asEventHandler;//< whether this runs as an event Handler or as a thread object

	};
}
//zavi::plugin

#endif /* PLUGINS_PSEUDOPLUGIN_HPP_ */
