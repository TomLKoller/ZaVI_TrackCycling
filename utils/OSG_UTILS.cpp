/*
 * OSG_UTILS.cpp
 *
 *  Created on: 04.02.2019
 *      Author: tomlucas
 */

#include "OSG_Utils.hpp"

namespace zavi
::osg_viz {

	osg::ref_ptr<osg::Geometry> pathCreator(double * states, int state_num, int state_size) {
		osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
		osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
		for (int i = 0; i < state_num; i++) {
			vertices->push_back(
					osg::Vec3(states[i * state_size + 0], states[i * state_size + 1], states[i * state_size + 2]));
		}
		geom->setVertexArray(vertices.get());
		osg::ref_ptr<osg::PrimitiveSet> prim = new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, vertices->size()-1);
		geom->addPrimitiveSet(prim);
		return geom;
	}

	osg::ref_ptr<osg::Geometry> pathShower(double * states, int state_num, int state_size) {
		osg::ref_ptr<osg::Geometry> geom =pathCreator(states,state_num,state_size);

		osg::ref_ptr<osg::Vec4Array> colors (new osg::Vec4Array());
		colors->push_back (osg::Vec4f (1.f,0.f,0.0f,1.0f));
		geom->setColorArray (colors.get(), osg::Array::BIND_OVERALL);
		osg::StateSet* state = geom->getOrCreateStateSet();
		state->setMode( GL_LIGHTING,osg::StateAttribute::OFF);
		geom->setStateSet(state);
		return geom;
	}

	osg::ref_ptr<osg::Geometry> pathShower(double * states, osg::ref_ptr<osg::Vec4Array> colors, int state_num, int state_size) {
			osg::ref_ptr<osg::Geometry> geom = pathCreator(states,state_num,state_size);
			geom->setColorArray (colors.get(), osg::Array::BIND_PER_VERTEX);
			osg::StateSet* state = geom->getOrCreateStateSet();
			state->setMode( GL_LIGHTING,osg::StateAttribute::OFF);
			//state->setMode( GL_BLEND, osg::StateAttribute::ON );
			//state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
			geom->setStateSet(state);

			return geom;
		}

	osg::ref_ptr<osg::Geometry> pathShower(double * states, double * norms, int state_num, int state_size) {

		osg::ref_ptr<osg::Vec4Array> colors (new osg::Vec4Array());
		for (int i = 0; i < state_num; i++) {
			colors->push_back (osg::Vec4f (norms[i],0.0f,0.0f,1.0f));
		}
		return pathShower(states,colors,state_num,state_size);
	}




}
