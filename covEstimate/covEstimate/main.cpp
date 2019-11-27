/*************************************************************************************************
* ----------------------------------COVARIANCE ESTIMATOR-----------------------------------------
***************************************************************************************************/
#define _USE_MATH_DEFINES
//c/c++
#include <iostream>
#include <string>
#include <cmath>

//opencv
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>

//openMVG
#include "openMVG/image/image_container.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/features/feature.hpp"
#include "openMVG/features/sift/SIFT_Anatomy_Image_Describer.hpp"
#include "openMVG/features/sift/sift_KeypointExtractor.hpp" //Modified version of the original openMVG - removed protected function

// Zeisl's method
#include "covEstimator.h"

//own
#include "covArgEvaluator.h"
#include "covOut.h"

using namespace std;
using namespace openMVG;
using namespace openMVG::image;
using namespace openMVG::features;
using namespace openMVG::features::sift;

// Define a feature and a container of features
using Feature_T = SIOPointFeature;
using Feats_T = vector<Feature_T>;

int main(int argc, char* argv[])
{
	string filename;
	Feats_T vec_feats;

	/*** Parsing input arguments ***/
	covArgEvaluator arg;
	arg.evaluate(argc, argv);

	/*** Loading input images and according keys ***/
	// Loading image
	filename.clear();
	filename.append(arg.imgDir).append(arg.imgFile);	
	const string imageFile = filename;

	Image<unsigned char> in;

	int res = ReadImage(imageFile.c_str(), &in);
	/*** Check if image is loaded fine ***/
	if(!res){
		cout << " Cov Error: Unable to load image from " << filename << "\n";
		exit(1);
	}

	/*** Loading the keypoints ***/
	filename.clear();
	filename.append(arg.imgDir).append(arg.keyFile);
	if (arg.verbose)
		cout << "Loading key points from file " << filename << endl;

	loadFeatsFromFile(filename, vec_feats);
	
	// Above is tested and working correctly

	/*** Creating image pyramid from openMVG***/

	// Convert to float
	Image<float> image;
	image = in.GetMat().cast<float>();
	// Create GSS
	const int supplementary_images = 3;

	HierarchicalGaussianScaleSpace octave_gen(6, 3, GaussianScaleSpaceParams(1.6f, 1.0f, 0.5f, supplementary_images));
	Octave octave;
	octave_gen.SetImage(image);

	/*** GSS and DoGs output to image files ***/
	cerr << "Octave computation started" << endl;
	uint8_t octave_id = 0;
	while (octave_gen.NextOctave(octave))
	{
		cerr << "Computed octave : " << to_string(octave_id) << endl;
		for (int i = 0; i < octave.slices.size(); ++i)
		{
			stringstream str;
			str << "gaussian_octave_" << to_string(octave_id) << "_" << i << ".png";
			WriteImage(str.str().c_str(), Image<unsigned char>(octave.slices[i].cast<unsigned char>()));
		}
		
		++octave_id;
	}


	/*** Creating image pyramid in IplImage ***/
	int octaves = 0, intervals = 3;
	IplImage*** dog_pyr;
	octaves = 6;
	dog_pyr = (IplImage***)calloc(octaves, sizeof(IplImage**));
	for (int i = 0; i < octaves; i++) {
		dog_pyr[i] = (IplImage**)calloc(intervals + supplementary_images - 1, sizeof(IplImage*));
	}
	// Read DoGs from  imagefiles 

	for (int o = 0; o < octaves; o++) {
		for (int i = 1; i < intervals + supplementary_images - 1; i++)
		{	
			stringstream str;
			str << "DoG_out_00" << to_string(o) << "_s" << "00" << i << ".png";
			dog_pyr[o][i] = cvLoadImage(str.str().c_str(), CV_LOAD_IMAGE_COLOR);
			cout << dog_pyr[o][i] << endl;
		}
	}
	string filename_out = filename + ".cov";
	ofstream& outfile = CovOut::initializeFile(filename_out);
	/*** Estimating the covariances for the keypoints ***/
	if (arg.verbose)
		cout << "COV Covariance estimation - Writing data to file " << filename << endl;
	CovEstimator estimator((const IplImage***)dog_pyr, octaves, intervals);
	for (unsigned int k = 0; k < vec_feats.size(); k++) {
		CvMat* cov = estimator.getCovAt(vec_feats[k].x(), vec_feats[k].y(), vec_feats[k].scale());	
		cout << cov << endl;
		// Output cov to file
		CovOut::write(outfile, cov);
	}
}
