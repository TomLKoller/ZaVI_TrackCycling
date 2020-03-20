/*
 * HeightMapSixdays.cpp
 *
 *  Created on: 11.02.2019
 *      Author: tomlucas
 */

#include "PenaltyMap.hpp"
#include <osg/ShapeDrawable>
#include <osgTerrain/Terrain>
#include <osg/Material>
#include <osgDB/ReadFile>
#include <osg/BlendFunc>
#include <osg/BlendEquation>
#include <stdlib.h>

namespace zavi
::estimator::pseudo_measurement {

	std::shared_ptr<PenaltyMap> getOrCreatePenaltyMap() {
		if(sixdays_penalty_map.get()==NULL)
		sixdays_penalty_map=std::make_shared<PenaltyMap>(cfg->lookup("penaltymap_file_name"));
		return sixdays_penalty_map;
	}

	PenaltyMap::PenaltyMap(const char * filename) {
		std::ifstream map(filename);
		if( map.is_open() )
		LOG(INFO) <<"Penalty map has been opened";
		else
		LOG(ERROR) << "Penalty map failed to open";
		int y=0;
		while(map.good()) {
			zavi::csv_reader::readLineToPointer(map, &penalty_map[width*y*3], width*3, ' ');
			y++;
		}

		if(y < height)
		LOG(FATAL) <<"Map does not match assumed size. Got height: " << y<< " But required: " <<height;
		grid=std::make_shared<ceres::Grid2D<double,3> >(penalty_map,0,height,0,width);     // x is the column index and y the row index
		interpolator=std::make_shared<ceres::BiCubicInterpolator<ceres::Grid2D<double,3> > >(*grid);
	}
	void PenaltyMap::addVisual(osg::ref_ptr<osg::Group> root) {
		osg::ref_ptr<osgTerrain::TerrainTile> terrain(new osgTerrain::TerrainTile());
		osg::ref_ptr<osgTerrain::Locator> locator(new osgTerrain::Locator());
		terrain->setLocator(locator);
		locator->setCoordinateSystemType(osgTerrain::Locator::CoordinateSystemType::PROJECTED);
		locator->setTransformAsExtents(0,0,width*MAP_PIXEL_TO_METER,height*MAP_PIXEL_TO_METER);
		//printf(locator->getCoordinateSystem());
		//printf("added locator");
		osg::ref_ptr<osg::HeightField> field(new osg::HeightField());
		field->allocate(width,height);
		// THe inner strides only works well for vectors, hence the penaltymap is vector mapped instead of matrix mapped to the heightlist
		Eigen::Map<Eigen::Matrix<float,width*height,1> > (&(field->getHeightList()[0]))=Eigen::Map<Eigen::Matrix<double,width*height,1>,0,Eigen::InnerStride<3> >(&penalty_map[2]).cast<float>();
		//field->setXInterval(0.1);
		//field->setYInterval(0.1);
		osg::ref_ptr<osgTerrain::HeightFieldLayer> layer(new osgTerrain::HeightFieldLayer);
		layer->setHeightField(field);
		layer->setLocator(locator);
		terrain->setElevationLayer(layer);

		std::string map_texture=cfg->lookup("map_texture");
		osg::ref_ptr<osg::Image> texture=osgDB::readImageFile(map_texture);
		osg::ref_ptr<osgTerrain::ImageLayer> image_layer(new osgTerrain::ImageLayer());
		image_layer->setLocator(locator);
		image_layer->setImage(texture);
		terrain->setColorLayer(0,image_layer);
		terrain->setBlendingPolicy(osgTerrain::TerrainTile::BlendingPolicy::ENABLE_BLENDING);
		osg::BlendEquation* blendEquation = new osg::BlendEquation(osg::BlendEquation::RGBA_MIN);
		blendEquation->setDataVariance(osg::Object::DYNAMIC);

		terrain->getOrCreateStateSet()->setAttributeAndModes(blendEquation,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);

		osg::BlendFunc *func = new osg::BlendFunc();
		func->setFunction(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		terrain->getOrCreateStateSet()->setAttributeAndModes(func);
		//printf(terrain->getBlendingPolicy());

		//visual->setShape(field);
		//visual->setColor(osg::Vec4d(0.5,0.5,0.5,0.5));
		//osg::StateSet* state = visual->getOrCreateStateSet();
		//state->setMode( GL_LIGHTING,osg::StateAttribute::OFF);
		terrain->getOrCreateStateSet()->setMode( GL_BLEND, osg::StateAttribute::ON );
		terrain->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

		root->addChild(terrain);
	}

	void PenaltyMap::addPointsWithHeight(osg::ref_ptr<osg::Group> root,double target_height, double tol, double step) {
		osg::ref_ptr<osg::Geometry> geometry (new osg::Geometry());

		osg::ref_ptr<osg::Vec3Array> vertices (new osg::Vec3Array());
		osg::ref_ptr<osg::Vec4Array> colors (new osg::Vec4Array());
		colors->push_back(osg::Vec4f(1.,0.,0.,1.));
		Eigen::Vector3d h;
		for(double x=0; x < width; x+=step) {
			for(double y=0; y < height; y+=step) {
				h=getPenaltyInCoords(x,y);
				if(h(2) <target_height + tol and h(2) > target_height - tol ) {
					vertices->push_back (osg::Vec3 (x*0.1,y*0.1, target_height));
				}

			}
		}
		geometry->setVertexArray (vertices.get());
		geometry->setColorArray (colors.get(), osg::Array::BIND_OVERALL);

		geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,vertices->size()));
		root->addChild(geometry);

	}

}
