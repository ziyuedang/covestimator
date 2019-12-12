#pragma once
//c++
#include <iostream>

//opencv
#include <opencv2/core/core_c.h>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>

//covUtils
#include "definitions.h"
using MatCv = cv::Mat;

class CovEstimator {

public:
	CovEstimator(vector<vector<MatCv>> pyr, int octvs, int intvls) {
		
		octaves = octvs;
		intervals = intvls;
		detectPyr = pyr;
		MatCv H(2, 2, CV_32FC1);
		MatCv cov(2, 2, CV_32FC1);
		MatCv evals(2, 1, CV_32FC1);
		MatCv evecs(2, 2, CV_32FC1);
	};


	const MatCv getImageFromPyramid(int octv, int intvl);
	MatCv getCovAt(float x, float y, float scale);


private:
	/*** Methods ***/
	MatCv hessian(const MatCv img, int r, int c);


	/*** Member variables ***/
	int type;
	int octaves, intervals;
	vector<vector<MatCv>> detectPyr;
	MatCv H;
	MatCv cov;
	MatCv evals;
	float ev1, ev2;
	MatCv evecs;
};
