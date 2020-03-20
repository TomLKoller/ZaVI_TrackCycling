/*
 * CSV_Reader.hpp
 *
 *  Created on: 16.01.2019
 *      Author: tomlucas
 */

#ifndef CSV_READER_HPP_
#define CSV_READER_HPP_

#include<string>
#include<vector>
#include <Eigen/Core>
#include<iostream>
#include<fstream>
#include <glog/logging.h>
namespace zavi
::csv_reader {

	/**
	 * Split a string at the delimiter char
	 * @param to_split the string to split
	 * @param delimiter the delimiter
	 * @return the splitted string as vector
	 */
	std::vector<std::string> splitString(std::string & to_split, const char delimiter) ;

	/**
	 * parse a string to a double vector
	 * @param vector the string vector
	 * @return the parsed values
	 */
	std::vector<double> parseFloatVector(std::vector<std::string> & vector);


	/**
	 * Parse an eigen matrix from a string vector
	 * @param vector the vector with all doubles
	 * @return the parsed values
	 */
	template<int size>
	Eigen::Matrix<double, size, 1> parseEigen(std::vector<std::string> & vector) {
		Eigen::Matrix<double, size, 1> floats;
		for (int i = 0; i < size; i++) {
			floats(i) = atof(vector[i].c_str());
		}
		return floats;
	}
	/**
	 * Reads a csv line to a double *
	 * @param csv the csv file ifstream
	 * @param result the pointer to put the result in (needs to be initialized)
	 * @param size  the line size/length
	 * @param delim the delimiter of the csv file
	 * @return whether the operation succeded
	 */
	bool readLineToPointer(std::ifstream &csv, double * result, const unsigned  int size,const char delim) ;
	/**
	 * Read a csv line to an eigen matrix
	 * @param csv the csv file stream
	 * @param result the matrix to store the line
	 * @param delim  the delimiter between values
	 * @return true if success
	 */
	template<int size>
	bool readLineToEigen(std::ifstream &csv, Eigen::Matrix<double,size,1> & result, const char delim='\t') {
		std::string line;
		if(csv.good()) {
			getline(csv,line);

			auto string_list=splitString(line,delim);
			if(string_list.size() < size) {
				LOG(WARNING)<< "Line of file did not contain enough data. Expected: "<<size <<"Got: " << string_list.size();
				return false;
			}
			result=parseEigen<size>(string_list);
			return true;
		}
		else {
			LOG(WARNING)<< "CSV File is empty";
			return false;
		}
	}

}
//zavi::csv_reader

#endif /* CSV_READER_HPP_ */
