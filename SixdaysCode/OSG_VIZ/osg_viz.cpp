/*
 * osg_viz.cpp
 *
 *  Created on: 17.07.2018
 *      Author: tomlucas
 */

#include "osg_viz.hpp"

#include "OSG_VIZ_Transform.hpp"

#include <osg/ShapeDrawable>
#include <osgGA/TrackballManipulator>

#include <ZaVI_Utils.hpp>

namespace zavi {
namespace osg_viz {
osg::ref_ptr<osg::Group> root;
osgViewer::Viewer viewer;

void initOSG() {
	root = new osg::Group();
	//osg::ref_ptr<osg::ShapeDrawable> plane(new osg::ShapeDrawable());
	//plane->setShape(new osg::InfinitePlane());
	//root->addChild(plane);
}

/**
 * Function to run osg viewer in another thread while keeping program able to react
 */
void osgStart() {
	viewer.setSceneData(root.get());
	if (!viewer.getCameraManipulator() && viewer.getCamera()->getAllowEventFocus()) {
		viewer.setCameraManipulator(new osgGA::TrackballManipulator());
	}
	viewer.getCamera()->setClearColor(osg::Vec4d(1., 1., 1., 1.));
	viewer.setReleaseContextAtEndOfFrameHint(false);
	viewer.setUpViewInWindow(0, 0, 700, 700, 0);
	viewer.getCameraManipulator()->setByInverseMatrix(osg::Matrix::lookAt(osg::Vec3d(37,18,150),osg::Vec3d(37.,18.,0),osg::Vec3d(0.,1.,0.)));
	viewer.run();
}

bool running() {
	return !viewer.done();
}

void addMapVisualizer(double width, double height, osg::ref_ptr<osg::Group> root) {
	osg::ref_ptr<osg::ShapeDrawable> map(new osg::ShapeDrawable());
	map->setColor(osg::Vec4d(0.5, 0.5, 0.5, 1.));
	map->setShape(new osg::Box(osg::Vec3d(width * 0.5, height * 0.5, 0), width, height, 0.01));
	root->addChild(map);
}

void addStateVisualiser(std::shared_ptr<zavi::plugin::StatePlugin> plugin, const osg::Vec4d & color,
		osg::ref_ptr<osg::Group> root, bool rotate) {
	osg::ref_ptr<osg::ShapeDrawable> estimate_box(new osg::ShapeDrawable());
	estimate_box->setShape(new osg::Box(osg::Vec3d(-0.75, 0, 0), 1.5, 2, 1));
	estimate_box->setColor(osg::Vec4d(0., 0., 0., 1.));
	osg::ref_ptr<osg::ShapeDrawable> estimate_box2(new osg::ShapeDrawable());
	estimate_box2->setShape(new osg::Box(osg::Vec3d(0.75, 0, 0), 1.5, 2, 1));
	estimate_box2->setColor(color);
	zavi::osg_viz::OSG_VIZ_Transform::addTransformedNode(estimate_box, plugin, root, rotate);
	zavi::osg_viz::OSG_VIZ_Transform::addTransformedNode(estimate_box2, plugin, root, rotate);
	//cordinate system
	osg::ref_ptr<osg::ShapeDrawable> estimate_box_x(new osg::ShapeDrawable());
	estimate_box_x->setShape(new osg::Box(osg::Vec3d(2, 0, 0), 1, 0.2, 0.2));
	estimate_box_x->setColor(osg::Vec4d(1., 0., 0., 1.));
	zavi::osg_viz::OSG_VIZ_Transform::addTransformedNode(estimate_box_x, plugin, root, rotate);

	osg::ref_ptr<osg::ShapeDrawable> estimate_box_y(new osg::ShapeDrawable());
	estimate_box_y->setShape(new osg::Box(osg::Vec3d(0, 1.5, 0), 0.2, 1, 0.2));
	estimate_box_y->setColor(osg::Vec4d(0., 1., 0., 1.));
	zavi::osg_viz::OSG_VIZ_Transform::addTransformedNode(estimate_box_y, plugin, root, rotate);

	osg::ref_ptr<osg::ShapeDrawable> estimate_box_z(new osg::ShapeDrawable());
	estimate_box_z->setShape(new osg::Box(osg::Vec3d(0., 0., 1.), 0.2, 0.2, 1));
	estimate_box_z->setColor(osg::Vec4d(0., 0., 1., 1.));
	zavi::osg_viz::OSG_VIZ_Transform::addTransformedNode(estimate_box_z, plugin, root, rotate);

}

void addStateVisualiser(std::shared_ptr<zavi::estimator::Estimator> estimator, const osg::Vec4d & color,
		osg::ref_ptr<osg::Group> root, bool rotate) {
	std::shared_ptr<zavi::plugin::StatePluginEstimator> plugin(new zavi::plugin::StatePluginEstimator(estimator));
	addStateVisualiser(plugin, color, root, rotate);
}

}		//osg_viz
}		//zavi

