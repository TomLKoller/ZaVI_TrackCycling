/*
 * FileIMUPlugin.cpp
 *
 *  Created on: 16.01.2019
 *      Author: tomlucas
 */

#include "FileIMUPlugin.hpp"
#include <CSV_Reader.hpp>
#include <ZaVI_Utils.hpp>
#include <ZaviConfig.hpp>
#include <glog/logging.h>
namespace zavi
::plugin {

	FileIMUPlugin::FileIMUPlugin(const char * filename, double skip_time,double end_time, bool batchwise):
	file(filename), skip(skip_time),end(end_time) , name_of_file(filename), batchwise(batchwise),added_states(0) {
		Timer::activateFileMode();
		acceleration_cov=Eigen::Matrix3d::Identity()*0.0004;
		orientation_cov=Eigen::Matrix3d::Zero();
		orientation_cov(0,0)=pow(0.75*M_PI/180,2);//from website
		orientation_cov(1,1)=pow(0.75*M_PI/180,2);//from website
		orientation_cov(2,2)=pow(1.5*M_PI/180,2);//from website
		angular_velocity_cov=Eigen::Matrix3d::Identity()*pow(0.1*M_PI/180.,2);
		acceleration_bias_cov=acceleration_bias_cov.Identity()*pow(0.1*0.91e-3,2);// bias stability from whitepaper
		acceleration_bias_corr=BIAS_MATRIX(std::initializer_list<double>( {1000,2000,1000,2000,1000,2000}).begin());
		angular_rate_bias_cov=angular_rate_bias_cov.Identity()*pow(10./180.*M_PI/3600.,2);
		angular_rate_bias_corr=Eigen::Vector3d(2000,2000,2000);
		// TODO Auto-generated constructor stub
		std::string line;
		std::getline(file, line);// skip first line with
		Eigen::Matrix<double, IMU_ROW_SIZE, 1> row=row.Zero();
		do {
			if(!zavi::csv_reader::readLineToEigen(file,row)) {
				LOG(FATAL)<< "file stop before the given start time";
			}
		}while(row(0)<skip_time);
		this->skip=row(0);
		LOG(INFO) << "Skip is " << skip;
		angular_velocity = row.block(4, 0, 3, 1);
		orientation = Eigen::Map<Eigen::Matrix<double,3,3> >(row.block(17, 0, 9, 1).data());

	}

	FileIMUPlugin::~FileIMUPlugin() {
		DLOG(INFO)<< "File imu got destroyed";
	}

	void FileIMUPlugin::run(double freq, bool (*running)()) {
		Timer loop(freq);
		//create noises
		Eigen::Matrix<double, IMU_ROW_SIZE, 1> row;
		double last_time=skip;
		double t_delta=1./(double) cfg->lookup("imu_freq");
		size_t calls=0;
		int drop_rate=cfg->lookup("data_drop_rate");
		if(drop_rate >0) {
			LOG(WARNING) << "Dropping out data of the file at " << drop_rate/10. << "\%";
			printf("Drop rate is set. Is this intended?");
		}
		while (running()) {
			if ((end ==-1. or row(0)<= end) and file.good()) {
				if(!zavi::csv_reader::readLineToEigen(file,row)) {
					continue;
				}
				if(rand() % 10<drop_rate) {
					continue;
				}
				calls++;
				acceleration = row.block(1, 0, 3, 1);
				angular_velocity = row.block(4, 0, 3, 1);
				for(size_t i =0; i < 3; i++) {
					if(acceleration(i) > 160.) {
						LOG(INFO) << row;
						LOG(WARNING) << "Found acceleration: " << acceleration(i) <<" It is over sensor limit. Found at: " <<row(0);
					}
					if(angular_velocity(i) > 2000./180.*M_PI) {
						LOG(INFO)<< row;
						LOG(WARNING) << "Found angular velocity: " << angular_velocity(i) << " It is over sensor limit. Found at: " <<row(0);
					}
				}
				magnetometer = row.block(7, 0, 3, 1);
				orientation = Eigen::Map<Eigen::Matrix<double,3,3> >(row.block(17, 0, 9, 1).data());
				acceleration_inc=row.block(10,0,3,1);
				double t_diff=row(0)-last_time-t_delta;
				while(round(t_diff/t_delta )>0) {
					data_valid=false;
					Timer::setFileTime(row(0)-t_diff);
					newDataAvailable();
					t_diff-=t_delta;
					added_states++;
				}
				data_valid=true;
				Timer::setFileTime(row(0));
				last_time=row(0);
				newDataAvailable();
				if(!batchwise)
				loop.wait();
			}
			else {
				batchReady();
				LOG(INFO)<< "Finished file "<< this->name_of_file << " at " << row(0) << "s";
				LOG(INFO) << "Read  " << calls << " valid states from file";
				LOG(INFO) << "Added " << added_states << " states that have no valid input";
				return;
			}
		}
	}

}
//namespace zavi::plugin

