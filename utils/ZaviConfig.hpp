/*
 * ZaviConfig.hpp
 *
 *  Created on: 27.03.2019
 *      Author: tomlucas
 */

#ifndef ZAVICONFIG_HPP_
#define ZAVICONFIG_HPP_

#include <libconfig.h++>
#include <memory>
extern std::shared_ptr<libconfig::Config> cfg;
void initConfig(const char * config_file) ;
std::shared_ptr<double> readTrialArray(const char * config_name, size_t size);
#endif /* ZAVICONFIG_HPP_ */
