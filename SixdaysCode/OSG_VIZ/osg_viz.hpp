/*
 * @file Visualization of ZaVi simulation
 * osg_viz.hpp
 *
 *  Created on: 17.07.2018
 *      Author: tomlucas
 */

#ifndef OSG_VIZ_OSG_VIZ_HPP_
#define OSG_VIZ_OSG_VIZ_HPP_

#include <osg/ref_ptr>
#include <osg/Group>
#include <osgViewer/Viewer>
#include <memory>
#include "../Plugins/state_plugin_base.hpp"
#include "../Estimators/Estimator.hpp"

namespace zavi {
namespace osg_viz {
extern osg::ref_ptr<osg::Group> root;
extern osgViewer::Viewer viewer;
/**
 * Creates root
 */
void initOSG();

/**
 * Starts the osg Viewer
 *
 * Only use after calling initOSG()
 * @see initOSG()
 */
void osgStart();

/**
 * Returns whether the viewer is still running
 * @return
 */
bool running();

/**
 * Adds a plane to show the map size
 * @param width width in meters
 * @param height height in meters
 * @param root  the group to add the node
 */
void addMapVisualizer(double width, double height, osg::ref_ptr<osg::Group> root = zavi::osg_viz::root);
/**
 * Adds a box with a colored front
 * @param plugin the plugin to read the state
 * @param color the color for the box tip
 * @param root the osg group
 * @param rotate whether to use rotation
 */
void addStateVisualiser(std::shared_ptr<zavi::plugin::StatePlugin> plugin,
		const osg::Vec4d & color = osg::Vec4d(0, 0, 1, 1), osg::ref_ptr<osg::Group> root = zavi::osg_viz::root,
		bool rotate = true) ;
/**
 * Adds a box with a colored front
 * @param estimator the estimator to provide the state
 * @param color the color for the box tip
 * @param root the osg group
 * @param rotate whether to use rotation
 */
void addStateVisualiser(std::shared_ptr<zavi::estimator::Estimator> estimator, const osg::Vec4d & color =
		osg::Vec4d(0, 0, 1, 1), osg::ref_ptr<osg::Group> root = zavi::osg_viz::root, bool rotate = true) ;
}     //osg_viz
}     //zavi

#endif /* OSG_VIZ_OSG_VIZ_HPP_ */
