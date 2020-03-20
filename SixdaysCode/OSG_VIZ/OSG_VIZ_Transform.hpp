/*
 * @file Contains all wrapper classes to visualize ode objects
 * ODE_OSG_VIZ_Transform.hpp
 *
 *  Created on: 19.07.2018
 *      Author: tomlucas
 */

#ifndef OSG_VIZ_OSG_VIZ_TRANSFORM_HPP_
#define OSG_VIZ_OSG_VIZ_TRANSFORM_HPP_
#include <memory>
#include <osg/PositionAttitudeTransform>
#include <ode/ode.h>
#include <osg/Callback>
#include <osg/ShapeDrawable>

#include "../Plugins/state_plugin_base.hpp"
#include "../Estimators/Estimator.hpp"
#include "../Plugins/state_plugin_estimator.hpp"

#include <OSG_Utils.hpp>

#include <memory>
namespace zavi {
namespace osg_viz {

/**
 * Abstract base class for classes visualizing ode Objects
 *
 * it can read the desired position for the visual from a basic StatePlugin
 * @see zavi::plugin::StatePlugin
 *
 * to use it just use addTransformedNode
 */
class OSG_VIZ_Transform: public ::osg::PositionAttitudeTransform {
public:

private:
	std::shared_ptr<zavi::plugin::StatePlugin> plugin;     //< shared pointer holding the ode part

	/**
	 * Callback to just align the position of an visual
	 */
	class positionCallback: public osg::NodeCallback {
	public:
		void operator()(osg::Node* node, osg::NodeVisitor* nv) ;
	};
	/**
	 * Callback to align osg object with ode object
	 */
	class odeCallback: public osg::NodeCallback {
	public:
		void operator()(osg::Node* node, osg::NodeVisitor* nv) ;
	};
public:

	/**
	 * Creates a new Transform and adds it to root
	 * @param node  the node which shall be transformed
	 * @param estimator estimator to read information
	 * @param root  the group where to add the new transform
	 * @param rotate whether to align rotation or not
	 */
	static void addTransformedNode(osg::ref_ptr<osg::Node> node, std::shared_ptr<zavi::estimator::Estimator> estimator,
			osg::ref_ptr<osg::Group> root, bool rotate = true);
	/**
	 * Creates a new Transform and adds it to root
	 * @param node  the node which shall be transformed
	 * @param plugin The StatePlugin to read Information
	 * @param root  the group where to add the new transform
	 * @param rotate whether to align rotation or not
	 */
	static void addTransformedNode(osg::ref_ptr<osg::Node> node, std::shared_ptr<zavi::plugin::StatePlugin> plugin,
			osg::ref_ptr<osg::Group> root, bool rotate = true);

	/**
	 * Creates the transform object
	 * @param plugin the Stateplugin to read pose information
	 * @param rotate whether to rotate the object
	 */
	OSG_VIZ_Transform(std::shared_ptr<zavi::plugin::StatePlugin> plugin, bool rotate = true);
	~OSG_VIZ_Transform();
};



}     //osg_viz

}     // zavi

#endif /* OSG_VIZ_OSG_VIZ_TRANSFORM_HPP_ */
