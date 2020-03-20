/*
 * HeightMapSixdays.hpp
 *
 *  Created on: 04.02.2019
 *      Author: tomlucas
 */

#ifndef ESTIMATORS_PSEUDOMEASUREMENTSHELPERCLASSES_PENALTYMAP_HPP_
#define ESTIMATORS_PSEUDOMEASUREMENTSHELPERCLASSES_PENALTYMAP_HPP_

#include <iostream>
#include <fstream>
#include <cmath>
#include <CSV_Reader.hpp>
#include "../../OSG_VIZ/osg_viz.hpp"
#include "../PseudoMeasurements.hpp"
#include <ZaviConfig.hpp>
#include <ceres/ceres.h>

#define MAP_PIXEL_TO_METER 0.1
#define MAP_METER_TO_PIXEL 10.


#include <ceres/cubic_interpolation.h>
namespace zavi
::estimator::pseudo_measurement {
	/**
	 * Pretty much the same as SixdaysHeightMap but returns the 3d distance instead of the height difference
	 */
	class PenaltyMap {
	public:
		static constexpr int width=759;
		static constexpr int height=369;

	private:

		static constexpr bool interpolate=true;
		double penalty_map[width*height*3];

		std::shared_ptr<ceres::Grid2D<double,3> > grid;
		std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<double,3> > > interpolator;

	public:
		PenaltyMap(const char * filename);

		/**
		 * Adds a heightmap node to a group
		 * @param root the group which shall contain the heightmap
		 */
		void addVisual(osg::ref_ptr<osg::Group> root);

		/**
		 * Add a filtered heightmap node to a group
		 * only points that have a height between height-tol  and height+tol are inserterd
		 * @param root the group which should contain the heightmap
		 * @param height the target height
		 * @param tol  the tolerance to accept points
		 * @param step  stride in x  and y direction
		 */

		void addPointsWithHeight(osg::ref_ptr<osg::Group> root,double height, double tol, double step);

		/**
		 * Get the penalty at a point
		 * @param mx  x in meter
		 * @param my  y in meter
		 * @return the penalty in meter
		 */

		template<typename T>
		Eigen::Matrix<T,3,1> getPenalty(const T & mx ,const T & my) {
			return getPenaltyInCoords(mx*MAP_METER_TO_PIXEL,my*MAP_METER_TO_PIXEL);
		}
		/**
		 * Get the height at a point in pixelspace
		 * @param px  x in pixels
		 * @param py  y in pixels
		 * @return height in meter
		 */

		template<typename T>
		Eigen::Matrix<T,3,1> getPenaltyInCoords(const T & px,const T & py) {
			Eigen::Matrix<T,3,1> result;
			interpolator->Evaluate(py,px,result.data());     // y is the row and x is the column
			return result;
		}

	}
	;

	inline std::shared_ptr<PenaltyMap> sixdays_penalty_map;     //!< static pointer to load map only once

	/**
	 * Always get a valid Penaltymap
	 * Creates one if not available
	 * @return sixdays_penalty_map
	 */

	std::shared_ptr<PenaltyMap> getOrCreatePenaltyMap();
	/**
	 * This Struct encapsulates the map penalty constrain
	 * It requires the estimator as prior
	 */
	template<typename estimator_type>
	struct PenaltyMapConstrain {
		SETUP_PSEUDO_MEASUREMENT(3)

		template<typename T>
		MEASURE_TYPE<T> operator()(const Eigen::Matrix<T, state_dim, 1> &state, void * prior) const {
			Eigen::Matrix<T, 3, 1> ground_pos = state.template block<3, 1>(0, 0);
			return MEASURE_TYPE<T>((ground_pos - getOrCreatePenaltyMap()->getPenalty(ground_pos(0, 0), ground_pos(1, 0))));

		}
		ADD_UKF_FUNCTION_PASSER
		template<typename T>
		static constexpr MEASURE_TYPE<T> getTarget() {
			return MEASURE_TYPE<T>::Zero();
		}
		static Eigen::Matrix<double, measure_dim, measure_dim> getNoise() {
			return Eigen::Matrix<double, measure_dim, measure_dim>::Identity() * 0.1;
		}
	};

}
//pseudo_measurements

#endif /* ESTIMATORS_PSEUDOMEASUREMENTSHELPERCLASSES_PENALTYMAP_HPP_ */
