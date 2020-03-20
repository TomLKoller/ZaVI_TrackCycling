/*
 * CSV_Reader.hpp
 *
 *  Created on: 16.01.2019
 *      Author: tomlucas
 */

#include "CSV_Reader.hpp"
#include "ZaVI_Utils.hpp"
namespace zavi
::csv_reader {

	bool readLineToPointer(std::ifstream &csv, double * result, const unsigned int size,const char delim) {
		std::string line;
		if (csv.good()) {
			getline(csv, line);
			auto string_list=splitString(line,delim);
			if(string_list.size() < size) {
				LOG(WARNING)<< "Line of file did not contain enough data. Expected: "<<size << " Got: " <<string_list.size();
				return false;
			}
			for (size_t i=0; i < string_list.size(); i++) {
				result[i]=atof(string_list[i].c_str());
			}
			return true;
		}
		else {
			LOG(WARNING)<< "CSV File is empty";
			return false;
		}
	}

	std::vector<std::string> splitString(std::string & to_split, const char delimiter) {
		std::stringstream split_stream(to_split);
		std::string segment;
		std::vector<std::string> seglist;
		while (std::getline(split_stream, segment, delimiter)) {
			if (segment != "")
			seglist.push_back(segment);
		}
		return seglist;

	}

	std::vector<double> parseFloatVector(std::vector<std::string> & vector) {
		std::vector<double> floats;
		for (std::string number : vector) {
			floats.push_back(atof(number.c_str()));
		}
		return floats;
	}

}
//zavi::csv_reader

