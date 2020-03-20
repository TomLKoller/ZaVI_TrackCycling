/*
 * OSG_VIZ_Transform.cpp
 *
 *  Created on: 05.02.2019
 *      Author: tomlucas
 */

#include "OSG_VIZ_Transform.hpp"

namespace zavi
::osg_viz {

	void OSG_VIZ_Transform::odeCallback::operator()(osg::Node* node, osg::NodeVisitor* nv) {
		OSG_VIZ_Transform *mt = dynamic_cast<OSG_VIZ_Transform*>(node);
		mt->setPosition(mt->plugin->getPositionOSG());
		mt->setAttitude(mt->plugin->getRotationOSG());
	}

	void OSG_VIZ_Transform::positionCallback::operator()(osg::Node* node, osg::NodeVisitor* nv) {
		OSG_VIZ_Transform *mt = dynamic_cast<OSG_VIZ_Transform*>(node);
		mt->setPosition(mt->plugin->getPositionOSG());
	}

	void OSG_VIZ_Transform::addTransformedNode(osg::ref_ptr<osg::Node> node, std::shared_ptr<zavi::estimator::Estimator> estimator,
			osg::ref_ptr<osg::Group> root, bool rotate ) {
		std::shared_ptr<zavi::plugin::StatePluginEstimator> plugin(new zavi::plugin::StatePluginEstimator(estimator));
		addTransformedNode(node, plugin, root, rotate);
	}

	void OSG_VIZ_Transform::addTransformedNode(osg::ref_ptr<osg::Node> node, std::shared_ptr<zavi::plugin::StatePlugin> plugin,
			osg::ref_ptr<osg::Group> root, bool rotate ) {
		osg::ref_ptr<zavi::osg_viz::OSG_VIZ_Transform> transform(new zavi::osg_viz::OSG_VIZ_Transform(plugin, rotate));
		transform->addChild(node);
		root->addChild(transform);
	}

	OSG_VIZ_Transform::OSG_VIZ_Transform(std::shared_ptr<zavi::plugin::StatePlugin> plugin, bool rotate ) :
	plugin(plugin) {
		if (rotate)
		this->setUpdateCallback(new odeCallback());
		else
		this->setUpdateCallback(new positionCallback());
	}
	OSG_VIZ_Transform::~OSG_VIZ_Transform() {
		DLOG(INFO) << "Destroyed transform node";
	}
}

