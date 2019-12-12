#pragma once
#include <string>
#include <fstream>
#include <cmath>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include "openMVG/features/feature.hpp"

class CovOut;
using namespace std;
using MatCv = cv::Mat;

class CovOut {
public:
	static ofstream& initializeFile(char* filename);
	static ofstream& initializeFile(string fn);
	static void write(ofstream& outfile, MatCv cov);
	static void closeFile(ofstream& outfile);
};