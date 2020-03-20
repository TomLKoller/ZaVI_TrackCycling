/*
 * state_plugin_base.hpp
 *
 *  Created on: 19.07.2018
 *      Author: tomlucas
 */

#ifndef PLUGINS_STATE_PLUGIN_BASE_HPP_
#define PLUGINS_STATE_PLUGIN_BASE_HPP_

#include <osg/Vec3d>
#include <osg/Quat>
namespace zavi{
namespace plugin{


/**
 * Base Class for all State plugins
 *
 *
 */
class StatePlugin{
public:
	/**
		 * Get the position as osg::Vec3D
		 * @return position of object
		 */
		virtual ::osg::Vec3d getPositionOSG()=0;

		/**
		 * Returns the rotation as osg::Quat
		 * @return the rotation
		 */
		virtual ::osg::Quat getRotationOSG()=0;

		virtual ~StatePlugin(){};

};

}//state_plugin
}//zavi




#endif /* PLUGINS_STATE_PLUGIN_BASE_HPP_ */
