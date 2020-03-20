/*
 * @file Helper file to visualize ODE Objets with OSG
 * OSG_utils.hpp
 *
 *  Created on: 18.07.2018
 *      Author: tomlucas
 */

#ifndef OSG_UTILS_HPP_
#define OSG_UTILS_HPP_

#include <osg/Vec3d>
#include <osg/Quat>
#include <osg/Geometry>
#include <osg/Array>

#include <Eigen/Geometry>

#include "ZaVI_Utils.hpp"

namespace zavi {
namespace osg_viz {

/**
 * Converts an Eigen to an OSG Vector
 * @param eigen a eigen 3d vector
 * @return an osg 3d vector
 */
inline osg::Vec3d eigenToOSGVector(const Eigen::Vector3d & eigen) {
	return osg::Vec3d(eigen[0], eigen[1], eigen[2]);

}
/**
 * converts an ode quaternion to osg
 * @param ode_quat the ode quaternion
 * @return an osg quaternion
 */
inline osg::Quat eigenToOSGQuat(const Eigen::Quaterniond &eigen) {
	return osg::Quat(eigen.x(), eigen.y(), eigen.z(), eigen.w());     //<ODE uses  [w,x,y,z] and OSG [x,y,z,w]
}

/**
 *  Creates an OSG geometry from a function (functor)
 * @param time_start starting time of the function
 * @param time_end  end time of the function
 * @param intervall intervall length (smoothness)
 * @param function  the function to evaluate
 * @return
 */
template<typename functor>
osg::ref_ptr<osg::Geometry> functorGeometry(double time_start, double time_end, double intervall, functor & function) {
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	double result[3];
	for (double t = time_start; t < time_end; t += intervall) {
		function(&t, (double *) result);
		vertices->push_back(osg::Vec3(result[0], result[1], result[2]));
	}
	geom->setVertexArray(vertices.get());
	osg::ref_ptr<osg::PrimitiveSet> prim = new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, vertices->size());
	geom->addPrimitiveSet(prim);
	return geom;
}


/**
 * Shows a taken path   (assumes that x y z is a 0 of the model
 * @param states double * to all states
 * @param colors array with colors for each state
 * @param state_num the amount of states
 * @param state_size the size of each state
 * @return
 */

osg::ref_ptr<osg::Geometry> pathShower(double * states, osg::ref_ptr<osg::Vec4Array> colors, int state_num, int state_size);

/**
 * Shows a taken path   (assumes that x y z is a 0 of the model
 * @param states double * to all states
 * @param state_num the amount of states
 * @param state_size the size of each state
 * @return
 */
osg::ref_ptr<osg::Geometry> pathShower(double * states, int state_num, int state_size);


/**
 * Shows a taken path   (assumes that x y z is a 0 of the model
 * @param states double * to all states
 * @param norms  the magnitudes of the vertices will be remapped to red color value
 * @param state_num the amount of states
 * @param state_size the size of each state
 * @return
 */
osg::ref_ptr<osg::Geometry> pathShower(double * states,double * norms, int state_num, int state_size);

}     //osg_viz

}     //zavi

#endif /* OSG_UTILS_HPP_ */
