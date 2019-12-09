#include "covOut.h"

ofstream& CovOut::initializeFile(char* filename) {
	ofstream* outfile = new ofstream(filename);

	*outfile << "# format:\n# COVXX COVXY COVYY" << endl;

	return *outfile;
}

ofstream& CovOut::initializeFile(string fn) {
	return initializeFile((char*)fn.c_str());
}

void CovOut::write(ofstream& outfile, MatCv* cov) {
	/* File format:
	covxx covxy covyy
	...
	*/


	// covariance information
	outfile << cov->at<float>(0, 0) << "\t";
	outfile << cov->at<float>(0, 1) << "\t";
	outfile << cov->at<float>(1, 1) << "\t";
	outfile << endl;
}

void CovOut::closeFile(ofstream& outfile) {
	outfile.close();
}