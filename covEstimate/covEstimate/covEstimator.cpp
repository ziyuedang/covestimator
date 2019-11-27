#include "CovEstimator.h"

using namespace std;

#define PI		3.14159265f

using namespace cv;

const Mat* CovEstimator::getImageFromPyramid(int octv, int intvl) {

	/* Interpolation between layers based on the parameter subscale would be possible here,
	but not implemented.
	*/
	return detecPyr[octv][intvl];
}



Mat* CovEstimator::getCovAt(float x, float y, float scale) {

	// Retrieve the octave and interval the feature was detected at
	int octv = 0, intv = 0, row = 0, col = 0;
	float subintv = 0.0f, subscale = 0.0f;


		// scale calculation: scl = sigma * 2 ^ ((octv*intlvs + inv)/intvls) * 1/ 2;
		float tmp = log(2 * scale / SIFT_SIGMA) / log(2.0f) * intervals;
		intv = ((int)cvRound(tmp) - 1) % intervals + 1;
		octv = ((int)cvRound(tmp) - 1) / intervals;
		subintv = tmp - (octv*intervals + intv);
		subscale = scale - SIFT_SIGMA / 2 * pow(2.0f, (octv*intervals + intv) / intervals);

		// location calculation: feat->x = ( c + xc ) * pow( 2.0, octv );
		col = cvRound(x / pow(2.0, octv - 1));
		row = cvRound(y / pow(2.0, octv - 1));


	// extract right image from pyramid
	const Mat* img = getImageFromPyramid(octv, intv);

	// determine hessan at that point and calculate and scale covariance matrix
	this->H = hessian(img, row, col);
	cvInvert(this->H, this->cov, CV_SVD_SYM);

	// Hessian is estimated at particular octave and interval, thus scaling needed, which
	// adapts for the octave and subinterval
	/*** with norm: */
	// double norm = cvNorm( cov, 0, CV_L2 );
	// cvScale( cov, cov, pow(scale, 2) / norm );
	/*** or fixed value: */

		cvScale(cov, cov, pow(2.0f, (octv + subintv / intervals) * 2) * 0.01);

	cvSVD(this->cov, this->evals, this->evecs);
	ev1 = evals->at<float>(0, 0);
	ev2 = evals->at<float>,(1, 0);
	if (ev1 < 0 && ev2 < 0) {
		ev1 = -ev1;
		ev2 = -ev2;
	}
	if (ev1 < ev2) {
		float tmp = ev1;
		ev1 = ev2;
		ev2 = tmp;
	}
	if (ev1 <= 0 || ev2 <= 0) {
		cout << "COV Eigenvalue of Hessian is negativ or zero(!)" << endl;
	}

	return this->cov;
}


Mat* CovEstimator::hessian(const Mat* dog, int row, int col) {

	int r, c;
	/*	r = row; c = col;
	float v   =   pixval32f( dog, r, c );
	float dxx = ( pixval32f( dog, r, c+1 ) +
	pixval32f( dog, r, c-1 ) - 2 * v );
	float dyy = ( pixval32f( dog, r+1, c ) +
	pixval32f( dog, r-1, c ) - 2 * v );
	float dxy = ( pixval32f( dog, r+1, c+1 ) -
	pixval32f( dog, r+1, c-1 ) -
	pixval32f( dog, r-1, c+1 ) +
	pixval32f( dog, r-1, c-1 ) ) / 4.0;

	*/	// ---
	float v, dxx = 0, dyy = 0, dxy = 0;
	float w[3][3] = { 0.0449f,    0.1221f,    0.0449f,
		0.1221f,    0.3319f,    0.1221f,
		0.0449f,    0.1221f,    0.0449f };

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++) {
			r = row + (j - 1); c = col + (i - 1);
			v = CV_IMAGE_ELEM(dog, float, r, c);
			dxx += w[i][j] * (CV_IMAGE_ELEM(dog, float, r, c + 1) +
				CV_IMAGE_ELEM(dog, float, r, c - 1) - 2 * v);
			dyy += w[i][j] * (CV_IMAGE_ELEM(dog, float, r + 1, c) +
				CV_IMAGE_ELEM(dog, float, r - 1, c) - 2 * v);
			dxy += w[i][j] * (CV_IMAGE_ELEM(dog, float, r + 1, c + 1) -
				CV_IMAGE_ELEM(dog, float, r + 1, c - 1) -
				CV_IMAGE_ELEM(dog, float, r - 1, c + 1) +
				CV_IMAGE_ELEM(dog, float, r - 1, c - 1)) / 4.0f;

		}

	cvmSet(H, 0, 0, -dxx);
	cvmSet(H, 0, 1, -dxy);
	cvmSet(H, 1, 0, -dxy);
	cvmSet(H, 1, 1, -dyy);

	return H;
}