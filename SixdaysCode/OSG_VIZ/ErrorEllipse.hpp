/*
 * ErrorEllipse.hpp
 *
 *  Created on: 01.08.2018
 *      Author: tomlucas
 */

#ifndef OSG_VIZ_ERRORELLIPSE_HPP_
#define OSG_VIZ_ERRORELLIPSE_HPP_

#include <osg/AutoTransform>
#include "../Plugins/state_plugin_estimator.hpp"
#include "../Estimators/Estimator.hpp"
#include <osg/PolygonMode>
#include <Eigen/Eigenvalues>
#include <math.h>

#include <Eigen_Utils.hpp>

#include <OSG_Utils.hpp>
#include "OSG_VIZ_Transform.hpp"
namespace zavi
::osg_viz {
	/**
	 * This class is used to visualize the covariance of an estimator
	 *
	 * just use addErrorEllipse() to show the covariance of an estimator
	 * @see addErrorEllipse
	 */
	class ErrorEllipse: public ::osg::PositionAttitudeTransform {

	public:
		/**
		 * the basic radius of the sphere
		 * @return the basic radius of the sphere
		 */
		inline osg::Vec3d getBaseRadModifier() {
			return base_rad;
		}
		/**
		 * the sigma intervall
		 * @return the sigma intervall
		 */
		inline double getSigma() {
			return sigma;
		}
	private:
		std::shared_ptr<zavi::plugin::StatePluginEstimator> plugin;     //< shared pointer holding the ode part
		osg::Vec3d base_rad;//< basic radius of sphere
		double sigma;//sigma intervall
		/**
		 * Callback to align osg object with ode object
		 */
		class estimatorCallback: public osg::NodeCallback {
		public:
			void operator()(osg::Node* node, osg::NodeVisitor* nv) {
				ErrorEllipse *mt = dynamic_cast<ErrorEllipse*>(node);
				auto cov = mt->plugin->getPositionError();
				//zavi::printf(cov);
				Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solv(cov);
				Eigen::Quaterniond quat(solv.eigenvectors());
				mt->setAttitude(zavi::osg_viz::eigenToOSGQuat(quat));
				Eigen::Vector3d scale = solv.eigenvalues().real().array().sqrt();
				mt->setScale(zavi::osg_viz::eigenToOSGVector(scale) * mt->getSigma() + mt->getBaseRadModifier());
				mt->setPivotPoint(osg::Vec3d(0, 0, 0));
				mt->setPosition(mt->plugin->getPositionOSG());
			}
		};
	public:
		/**
		 * Creates a new Transform and adds it to root
		 * @param node  the node which shall be transformed
		 @param estimator estimator to read information
		 * @param root  the group where to add the new transform
		 * @param base_rad the basic radius of the ellipse osg::Vec3d(base_rad,base_rad,base_rad)
		 * @param sigma the sigma intervall default 1
		 */
		static void addErrorEllipse(std::shared_ptr<zavi::estimator::Estimator> estimator,
				osg::ref_ptr<osg::Group> root=zavi::osg_viz::root, double base_rad=1, double sigma = 1) {
			std::shared_ptr<zavi::plugin::StatePluginEstimator> plugin(new zavi::plugin::StatePluginEstimator(estimator));
			addErrorEllipse(plugin,root,base_rad,sigma);
		}
		/**
		 * Creates a new Transform and adds it to root
		 * @param node  the node which shall be transformed
		 * @param plugin The StatePlugin to read Information
		 * @param root  the group where to add the new transform
		 * @param base_rad the basic radius of the ellipse osg::Vec3d(base_rad,base_rad,base_rad)
		 * @param sigma the sigma intervall default 1
		 */
		static void addErrorEllipse(
				std::shared_ptr<zavi::plugin::StatePluginEstimator> plugin,
				osg::ref_ptr<osg::Group> root=zavi::osg_viz::root, double base_rad=1,double sigma=1) {
			osg::ref_ptr<osg::ShapeDrawable> estimate_sphere(new osg::ShapeDrawable());
			estimate_sphere->setShape(new osg::Sphere(osg::Vec3d(0,0,0),1));
			osg::PolygonMode* polymode = new osg::PolygonMode;
			polymode->setMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::LINE);
			estimate_sphere->getOrCreateStateSet()->setAttributeAndModes(polymode,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
			osg::ref_ptr<zavi::osg_viz::ErrorEllipse> transform(
					new zavi::osg_viz::ErrorEllipse(plugin,base_rad,sigma));
			transform->addChild(estimate_sphere);
			//zavi::osg_viz::OSG_VIZ_Transform::addTransformedNode(transform,plugin,root,false);
			transform->setReferenceFrame(osg::Transform::ReferenceFrame::RELATIVE_RF);
			root->addChild(transform);
		}

		/**
		 * Creates the transform object
		 * @param plugin the Stateplugin to read pose information
		 * * @param base_rad the basic radius of the ellipse
		 */
		ErrorEllipse(std::shared_ptr<zavi::plugin::StatePluginEstimator> plugin, double base_rad, double sigma) :
		plugin(plugin),base_rad(osg::Vec3d(base_rad,base_rad,base_rad)),sigma(sigma) {
			this->setUpdateCallback(new estimatorCallback());

		}
		~ErrorEllipse()
		{
		}
	}
	;

}
//zavi::osg_viz

#endif /* OSG_VIZ_ERRORELLIPSE_HPP_ */
