/*
 * FileIMUPlugin.h
 *
 *  Created on: 16.01.2019
 *      Author: tomlucas
 */

#ifndef PLUGINS_FILEIMUPLUGIN_HPP_
#define PLUGINS_FILEIMUPLUGIN_HPP_
#define IMU_ROW_SIZE 26
#define OFFSET_TIL_START 5
#include "imu_plugin.hpp"

#include <iostream>
#include <fstream>

namespace zavi
::plugin {
	class FileIMUPlugin: public IMU_Plugin {
		typedef Eigen::Matrix<double,6,1> BIAS_MATRIX;
	public:
		FileIMUPlugin(const char * filename, double skip_time=0, double end_time=-1.0, bool batchwise=false);
		virtual ~FileIMUPlugin();
		void run(double freq, bool (* running)());
		virtual bool can_batchwise() {return true;}
		Eigen::Vector3d acceleration_inc;
	protected:
		std::ifstream file;
		double skip;
		double end;
		const char * name_of_file;
		const bool batchwise; // whether this one is in replay mode
		size_t added_states;
	};

}
/* namespace zavi::plugin */

#endif /* PLUGINS_FILEIMUPLUGIN_HPP_ */
