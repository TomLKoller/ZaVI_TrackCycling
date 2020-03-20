/*
 * LaserBarrier.cpp
 *
 *  Created on: 18.02.2019
 *      Author: tomlucas
 */

#include "LaserBarrier.hpp"

#include <osg/PositionAttitudeTransform>
#include <osg/LineWidth>
#include <OSG_Utils.hpp>
#include <ZaVI_Utils.hpp>
#include <ZaviConfig.hpp>
#include <iostream>
#include <fstream>
#include <CSV_Reader.hpp>
#include <cassert>
#include <Eigen_Utils.hpp>
#include <glog/logging.h>

#define IMU_FREQ 100.
#define MIBI_SECONDS_PER_SECOND 1024.
namespace zavi
::sixdays {

	LaserBarrier::LaserBarrier(int number, const Eigen::Vector3d & laser_pos, const Eigen::Vector3d & detector_pos,const char * filename,osg::ref_ptr<osg::Group> group):number(number),laser_pos(laser_pos),detector_pos(detector_pos), laser_vector(detector_pos-laser_pos),
	laser_box(new osg::Geometry()) {
		addLaserVisual(group);
		readSeesTargetFile(filename);
        static libconfig::Setting & conf=cfg->lookup((const char *)cfg->lookup("trial"));
		bike_info.imu_to_back=Eigen::Vector3d(0.- (double)conf.lookup("wheel_radius"),0,0);
		bike_info.imu_to_front=Eigen::Vector3d((double)conf.lookup("wheel_dist")+ (double)conf.lookup("wheel_radius"),0,0);
	}

	std::shared_ptr<LaserBarrier> LaserBarrier::createLaserBarrier(int number,const Eigen::Vector3d & laser_pos,const Eigen::Vector3d & detector_pos,const char* filename, osg::ref_ptr<osg::Group> group) {
		std::shared_ptr<LaserBarrier> shptr(new LaserBarrier(number,laser_pos,detector_pos,filename,group));
		all_barriers.push_back(std::weak_ptr<LaserBarrier>(shptr));
		return shptr;
	}

	osg::ref_ptr<osg::Vec4Array> LaserBarrier::getVerticeColoring(int state_size) {
		osg::ref_ptr<osg::Vec4Array> colors (new osg::Vec4Array());
		LOG(INFO)<<"number of barriers: "<< all_barriers.size() << " number of states: "<<state_size;
		for(unsigned int i=0; i < state_size; i++) {
			float r=0.f,g=0.f,b=0.f;
			for(std::weak_ptr<LaserBarrier> weak_laser: all_barriers) {
				auto laser=weak_laser.lock();
				bool exp_target=std::get<bool>(laser->expects_target[i]);
				int sees_target_index=std::get<double>(laser->expects_target[i])*IMU_FREQ;
				//LOG(INFO) << "sees target index" <<sees_target_index << "time " << std::get<double>(laser->expects_target[i]) << "expected target " << exp_target  ;
				if(sees_target_index < laser->sees_target.size()) {
					bool sees_target_currently=laser->sees_target[sees_target_index] >0;
					if(sees_target_currently and exp_target) {
						g+=1.;
						//DLOG(INFO) << "sees target and expected it";
					}
					else if(sees_target_currently) {
						r+=1.;
						//DLOG(INFO)<<"sees target";
					}
					else if(exp_target)
					b+=1.;
				}
				else if(exp_target)
				b+=1.;
			}
			colors->push_back(osg::Vec4f(r,g,b,1.0f));
//					colors->push_back(osg::Vec4f(r/all_barriers.size(),g/all_barriers.size(),b/all_barriers.size(),1.0f));

		}

		return colors;
	}

	LaserBarrier::~LaserBarrier() {
		DLOG(INFO)<< "Writing LaserBarrier data";
		std::stringstream filename;
		filename << "Laser_"<<number <<"_expected.csv";
		std::ofstream file(filename.str());
		for(unsigned int i=0; i < expects_target.size(); i++) {
			std::stringstream line;
			line <<std::get<double>(expects_target[i]) << " " <<std::get<bool>(expects_target[i])<< "\n";
			file.write(line.str().c_str(),line.str().length());
		}
		file.flush();
		file.close();
	}

	void LaserBarrier::addLaserVisual(osg::ref_ptr<osg::Group> group) {
		osg::LineWidth* linewidth = new osg::LineWidth();
		linewidth->setWidth(cfg->lookup("laser_width"));
		osg::ref_ptr<osg::Vec3Array> vertices (new osg::Vec3Array());
		osg::ref_ptr<osg::Vec4Array> colors (new osg::Vec4Array());
		colors->push_back(osg::Vec4f(1.f,0,0,1.f));
		vertices->push_back(osg_viz::eigenToOSGVector(laser_pos));
		vertices->push_back(osg_viz::eigenToOSGVector(detector_pos));
		laser_box->setVertexArray (vertices.get());
		laser_box->setColorArray(colors.get(), osg::Array::BIND_OVERALL);
		laser_box->getOrCreateStateSet()->setAttributeAndModes(linewidth, osg::StateAttribute::ON);
		laser_box->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,0,vertices->size()));
		laser_box->getOrCreateStateSet()->setMode( GL_LIGHTING,osg::StateAttribute::OFF);
		group->addChild(laser_box);

	}

	void LaserBarrier::readSeesTargetFile(const char * filename) {
		Eigen::Matrix<double, 2, 1> row;
		DLOG(INFO)<< "Reading laser file " <<filename;
		std::ifstream file(filename);
		std::vector<int> raw_data;
		while(file.good()) {
			if(!zavi::csv_reader::readLineToEigen(file,row, ' ')) {
				raw_data.push_back(false);
				continue;
			}
			//if(row(0)>skip_time*MIBI_SECONDS_PER_SECOND)
			raw_data.push_back((int) row(1));

		}
		double overall_time=raw_data.size()/MIBI_SECONDS_PER_SECOND;
		double subsampled_time=0.0;
		double current_time;
		double time_diff_half=0.5/IMU_FREQ;
		int hits=0;
		for(unsigned int i=0; i < raw_data.size(); i++) {
			current_time=i/MIBI_SECONDS_PER_SECOND;
			if(current_time >= subsampled_time+time_diff_half) {
				sees_target.push_back(hits);
				hits=0;

				subsampled_time+=time_diff_half*2;
			}
			hits+=raw_data[i];
		}
		DLOG(INFO)<< "read " << sees_target.size() <<" samples from file";

	}

	void LaserBarrier::operator()(void* callbacker) {
		//DLOG(INFO)<< "LaserBarrier is called";
		estimator::Estimator * bike=static_cast<estimator::Estimator *>(callbacker);
		Eigen::Matrix3d Q = bike->getEstimatedOrientation().toRotationMatrix();
		expects_target.push_back(std::make_tuple(Timer::getReplayTime(),eigen_util::lineIntersectsPlane<double>(bike->getEstimatedPosition()+Q*bike_info.imu_to_back,Q*(bike_info.imu_to_front-bike_info.imu_to_back),laser_pos-Eigen::Vector3d(0,0,100),laser_vector,Eigen::Vector3d(0,0,200))));
		//plot::liveStateDraw<12>(Eigen::Vector3d(expects_target.back(),sees_target[time],-1), "Laser Barrier");
		/*if(eigen_util::lineIntersectsPlane<double>(bike->getEstimatedPosition()+Q*bike_info.imu_to_back,Q*(bike_info.imu_to_front-bike_info.imu_to_back),laser_pos-Eigen::Vector3d(0,0,100),laser_vector,Eigen::Vector3d(0,0,200))) {
		 if (sees_target[time]) {
		 printf("TP");
		 laser_box->setColor(osg::Vec4d(0., 1., 0., 1.));     //TP
		 } else {
		 printf("FP");
		 laser_box->setColor(osg::Vec4d(1., 0., 0., 1.));     // FP

		 }
		 return;

		 }
		 if (sees_target[time]) {
		 printf("FN");
		 laser_box->setColor(osg::Vec4d(0., 0., 1., 1.));     // FN
		 } else {
		 laser_box->setColor(osg::Vec4d(0., 0., 0., 1.));     // TN
		 }*/

	}

}
