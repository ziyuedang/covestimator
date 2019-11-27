#pragma once
//c++
#include <iostream>

//opencv
#include <opencv2/core/core_c.h>
#include <opencv2/core/mat.hpp>
//covUtils
#include "definitions.h"


class CovEstimator {

public:
	CovEstimator(const cv::Mat*** pyr, int octvs, int intvls) {
		
		octaves = octvs;
		intervals = intvls;
		detecPyr = pyr;
		cv::Mat H(2, 2, CV_32FC1);
		cv::Mat cov(2, 2, CV_32FC1);
		cv::Mat evals(2, 1, CV_32FC1);
		cv::Mat evecs(2, 2, CV_32FC1);
	};


	const cv::Mat* getImageFromPyramid(int octv, int intvl);

	cv::Mat* getCovAt(float x, float y, float scale);


private:
	/*** Methods ***/
	cv::Mat* hessian(const cv::Mat* img, int r, int c);

	/*  Linear interpolation
	target  - the target point, 0.0 - 1.0
	v       - a pointer to an array of size 2 containg the two values
	*/
	float linearInterp(float target, float v[]) {
		return (float)(target*(v[1]) + (1.0f - target)*(v[0]));
	}

	/*  BiLinear interpolation, linear interpolation in 2D
	target  - a 2D point (X,Y)
	v       - an array of size 4 containg values left to right, top to bottom
	cost: performs 3 linear interpolations
	*/
	float bilinearInterp(float target[], float v[]) {
		float v_prime[2] = { linearInterp(target[0], &(v[0])),
			linearInterp(target[0], &(v[2])) };
		return linearInterp(target[1], v_prime);
	}

	/*  TriLinear interpolation, linear interpolation in 3D
	target  - a 3D point (X,Y,Z)
	v       - an array of size 8 containg the values of the 8 corners
	of a cube defined as two faces: 0-3 face one (front face)
	4-7 face two (back face)
	cost: 7 linear interpolations
	*/
	float trilinearInterp(float target[], float v[]) {
		float v_prime[2] = { bilinearInterp(&(target[0]), &(v[0])),
			bilinearInterp(&(target[0]), &(v[4])) };
		return linearInterp(target[2], v_prime);
	}


	/*** Member variables ***/
	int type;
	const cv::Mat*** detecPyr;
	int octaves, intervals;

	cv::Mat* H;
	cv::Mat* cov;
	cv::Mat* evals;
	float ev1, ev2;
	cv::Mat* evecs;
};
