/*@file contains different functions
 *
 * use as guidance to implement new functions
 * Functors.hpp
 *
 *  Created on: 08.08.2018
 *      Author: tomlucas
 */

#ifndef SETUPS_FUNCTORS_HPP_
#define SETUPS_FUNCTORS_HPP_
#include<random>

namespace zavi
::functions {
	/**
	 * base struct for functors
	 *
	 * use give real to determine whether in ceres solver
	 * can be used to add random walk error on function without telling the solver
	 */
	struct Functor {
		//expand noise is used when a noise increment shall be applied ( in general when you want a new measurement) use randomWalkFunctor to implement
		//template<typename T>
		//virtual bool operator()(const T time, T* result,bool expand_noise)const ; //needed but no compiler support for virtual template

		bool give_real;//< determines whether to give the real function value or an approximation
		unsigned int DOF;
		Functor(int DOF):give_real(false),DOF(DOF) {}
	};
	/**
	 * An Object to store a random walk pattern
	 *
	 * can be used multiple times in a functor
	 */
	struct RandomWalkFunctor {

		double sigma,top,bottom;     //< sigma and top bottom limit of randomwalk
		double walk;//< the current walk value
		std::normal_distribution<double> random;//< random noise variables
		std::default_random_engine generator;//< random generator
		RandomWalkFunctor():sigma(0),top(0),bottom(0),walk(0) {}
		RandomWalkFunctor(double sigma,double top,double bottom):sigma(sigma),top(top),bottom(bottom),walk(0) {
			random=std::normal_distribution<double>(0.,sigma);
		}
		/**
		 * Add a random sample to the walk
		 *
		 * keep limits
		 */
		void expandNoise() {
			walk+=random(generator);
			if (walk > top)
			walk=top;
			if (walk < bottom)
			walk=bottom;
		}

	};

	/**
	 * A cosine function
	 */
	struct SixdaysMidwayFunctor: public Functor {
		SixdaysMidwayFunctor():Functor(1) {}
		template<typename T>
		bool operator()(const T * const time, T* result,bool expand_noise=false)const {
			T t=time[0]-10.; // correct starting line
			const double r=13.95;
			const double l=170;
			t = fmod(t,l);
			if (t < T(0))
			t += l;
			const double center_x=37.95, center_y=18.45;
			const double rpi=r*M_PI;
			const double s=(l-2*rpi)/2;
			if(t <rpi)
			result[0]=r*cos(t/r+M_PI/2)+center_x-s/2;
			result[1]=r*sin(t/r+M_PI/2)+center_y;
			if(t >rpi and t < rpi+ s) {
				result[0]=t-rpi+center_x-s/2;
				result[1]=center_y-r;
			}
			if(t > rpi+ s and t < 2*rpi+s) {
				result[0]=r*cos((t-rpi-s)/r-M_PI/2)+center_x+s/2;
				result[1]=r*sin((t-rpi-s)/r-M_PI/2)+center_y;

			}
			if(t >2*rpi+s) {
				result[0]=-(t-2*rpi-s)+center_x+s/2;
				result[1]=center_y+r;
			}

			result[2]=T(1.0);
			return true;
		}
	};

	/**
	 * A cosine function
	 */
	struct CosFunctor: public Functor {
		CosFunctor():Functor(1) {}
		template<typename T>
		bool operator()(const T * const time, T* result,bool expand_noise=false)const {
			result[0]=T(0);
			result[1]=time[0];
			result[2]=2.*cos(time[0]);
			return true;
		}
	};
	/**
	 * a circle trajectory
	 *
	 * with possible noise in y axis
	 */
	struct CircleFunctor: public Functor {
		RandomWalkFunctor walker;
		CircleFunctor():Functor(1) {
			walker=RandomWalkFunctor(0.00,0.5,-0.5);
		}
		template<typename T>
		bool operator()(const T * const time, T* result)const {
			result[0]=2.*sin(time[0]);
			result[1]=2.*cos(time[0]);
			result[2]=T(0)+walker.walk;
			return true;
		}
		template<typename T>
		bool operator()(const T * const time, T* result,bool expand_noise) {
			if(expand_noise)
			walker.expandNoise();
			this->operator ()(time,result);
			return true;
		}

	};

}
//zavi::functions

#endif /* SETUPS_FUNCTORS_HPP_ */
