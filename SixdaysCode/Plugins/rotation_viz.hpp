/*
 * @file
 * state_plugin_estimator.hpp
 *
 *  Created on: 26.07.2018
 *      Author: tomlucas
 */

#ifndef PLUGINS_STATE_PLUGIN_ROTATION_VIZ_ESTIMATOR_HPP_
#define PLUGINS_STATE_PLUGIN_ROTATION_VIZ_ESTIMATOR_HPP_

#include "state_plugin_base.hpp"
#include "../Estimators/Estimator.hpp"
#include <zavi/utils/OSG_Utils.hpp>

#include <Eigen/Core>
#include <osg/Vec3d>
#include <osg/Quat>
namespace zavi::plugin{


	class StatePluginRotationViz : public StatePlugin{
public:

		StatePluginRotationViz(){

				}
				inline ::osg::Vec3d getPositionOSG(){
						return osg::Vec3d(0,0,0);}
				inline Eigen::Matrix3d getPositionError(){
						throw "getPositionError: Not implemented in StatePluginRotationViz";
				}


				inline ::osg::Quat getRotationOSG(){
					static double r =0,p=0,y=1;
					double rd=0.0001;
					double pd=-rd*cos(p)*(sin(y)-cos(y))/(sin(y)+cos(y));
					double yd=-pd*sin(y)-rd*(cos(p)*cos(y)+sin(p));
					r+=rd;
					p+=pd;
					y+=yd;
					Eigen::Matrix3d ry, rp,rr, rotation;
					ry << cos(y), -sin(y), 0,sin(y), cos(y), 0 ,0,0,1;
					rp << cos(p), 0,sin(p),0,1,0,-sin(p),0,cos(p);
					rr <<1,0,0,0,cos(r),-sin(r),0,sin(r),cos(r);
					rotation=ry*rp*rr;

					return zavi::osg_viz::eigenToOSGQuat(Eigen::Quaterniond(rotation));
				}

private:

	};

}//zavi::plugin




#endif /* PLUGINS_STATE_PLUGIN_ROTATION_VIZ_ESTIMATOR_HPP_ */
