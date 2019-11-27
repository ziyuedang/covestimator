#pragma once
#include <string>
#include <fstream>
#include <cmath>
#include <iostream>
#include <opencv2/core/core_c.h>
#include "openMVG/features/feature.hpp"

class CovOut;
using namespace std;

class CovOut {
public:
	static ofstream& initializeFile(char* filename);
	static ofstream& initializeFile(string fn);
	static void write(ofstream& outfile, CvMat* cov);
	static void closeFile(ofstream& outfile);
};