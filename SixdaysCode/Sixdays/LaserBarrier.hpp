/*
 * LaserBarrier.hpp
 *
 *  Created on: 18.02.2019
 *      Author: tomlucas
 */

#ifndef SIXDAYS_LASERBARRIER_HPP_
#define SIXDAYS_LASERBARRIER_HPP_

#include <threaded_object.hpp>
#include "Eigen/Core"
#include "../Estimators/Estimator.hpp"
#include <Callbacker.hpp>
#include "BikeInfo.hpp"
#include "../OSG_VIZ/osg_viz.hpp"
#include <osg/ShapeDrawable>

namespace zavi
::sixdays {

	class LaserBarrier:  public Callback {
	private:
		int number=-1;
		Eigen::Vector3d laser_pos,detector_pos;     // position of the laser emitter and the detector
		Eigen::Vector3d laser_vector;
		std::vector<int> sees_target;// complete trial data when this laser barrier was activated by a bike
		std::vector<std::tuple<double,bool >> expects_target;
		BikeInfo bike_info;
		osg::ref_ptr<osg::Geometry> laser_box;
		static inline std::vector<std::weak_ptr<LaserBarrier> > all_barriers;
	public:

		virtual ~LaserBarrier();
		virtual void operator() (void* callbacker) ;
		static std::shared_ptr<LaserBarrier> createLaserBarrier(int number,const Eigen::Vector3d & laser_pos,const  Eigen::Vector3d & detector_pos,const char* filename, osg::ref_ptr<osg::Group> group=osg_viz::root);
		static osg::ref_ptr<osg::Vec4Array> getVerticeColoring(int state_size);
	protected:
		LaserBarrier(int number,const Eigen::Vector3d & laser_pos,const  Eigen::Vector3d & detector_pos,const char* filename, osg::ref_ptr<osg::Group> group=osg_viz::root);
	private:
		void addLaserVisual(osg::ref_ptr<osg::Group> group=osg_viz::root);
		void readSeesTargetFile(const char * filename);

	};

}

#endif /* SIXDAYS_LASERBARRIER_HPP_ */
