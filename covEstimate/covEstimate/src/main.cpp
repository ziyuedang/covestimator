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

//openMVG
#include "openMVG/image/image_container.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/features/feature.hpp"
#include "openMVG/features/sift/SIFT_Anatomy_Image_Describer.hpp"
#include "openMVG/features/sift/sift_KeypointExtractor.hpp" 

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
using MatCv = cv::Mat;

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

	/*** GSS output to image files ***/
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

	/*** Read GSS and compute Difference of Gaussian (DoG) ***/

	int octaves = 6, intervals = 3;
	vector<vector<MatCv>> dog_pyr(octaves, vector<MatCv>(intervals + supplementary_images - 1));
	for (int o = 0; o < octaves; o++) {
		for (int i = 1; i < intervals + supplementary_images - 1; i++) {
			stringstream str1;
			stringstream str2;
			str1 << "gaussian_octave_" << to_string(o) << "_" << i - 1 << ".png";
			str2 << "gaussian_octave_" << to_string(o) << "_" << i << ".png";
			MatCv dog_prev = cv::imread(str1.str().c_str(), cv::IMREAD_UNCHANGED);
			MatCv dog_cur = cv::imread(str2.str().c_str(), cv::IMREAD_UNCHANGED);
			dog_pyr[o][i] = dog_cur - dog_prev;			
		}
	}
	string filename_out = filename + ".cov";
	ofstream& outfile = CovOut::initializeFile(filename_out);
	/*** estimating the covariances for the keypoints ***/
	if (arg.verbose)
		cout << "cov covariance estimation - writing data to file " << filename << endl;
	CovEstimator estimator(dog_pyr, octaves, intervals);
	for (int k = 0; k < vec_feats.size(); k++) {
		MatCv cov = estimator.getCovAt(vec_feats[k].x(), vec_feats[k].y(), vec_feats[k].scale());	
		cout << cov << endl;
		cout << "size of cov is: " << cov.size() << endl;
//		// Output cov to file
//		CovOut::write(outfile, cov);
	}
}

