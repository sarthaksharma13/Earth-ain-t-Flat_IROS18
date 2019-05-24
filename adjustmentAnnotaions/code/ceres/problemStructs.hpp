#include <ceres/ceres.h>
#include <ceres/rotation.h>

class BundleAdjustmentProblem{


public:
	double getNumViews() { return numViews_;}
	double getNumPoints() { return numPoints_;}
	double *get3DPoints() { return Xs_; }
	double *get2DPoints() { return xs_; }
	double *getK() { return K_;}
	double *getRs() { return Rs_;}
	double *getts() { return ts_;}

	bool loadFile(const char *fileName){

		
		FILE *fptr = fopen(fileName, "r");
		if(fptr == NULL){
			return false;
		}

		// Read mumber of views and points.
		fscanfOrDie(fptr,"%d",&numViews_);
		fscanfOrDie(fptr,"%d",&numPoints_);

		// Read K matrix
		K_ = new double[9];
		for(int i=0;i<9;++i){
			fscanfOrDie(fptr,"%lf",K_ + i);
		}
	
		//Read 3D points : rougly initilaized via triangulation.
		Xs_ = new double[3*numPoints_];
		for(int i=0;i<numPoints_;i++){
			for(int j=0;j<3;j++)
				fscanfOrDie(fptr,"%lf",Xs_ + 3*i + j);
		}

	/*	for(int i=0;i<numPoints_;i++){
			std::cout << Xs_[3*i+0 ] << " " << Xs_[3*i+1] << " " << Xs_[3*i+2] << std::endl;
		}
	*/	
		// Read R and t
		Rs_=new double[9*numViews_];
		 for(int i=0;i<numViews_;i++){
		 	for(int j=0;j<9;j++)
		 		fscanfOrDie(fptr,"%lf",Rs_ + 9*i + j);
		}

	/*	for(int i=0;i<numViews_;i++){
			std::cout << Rs_[9*i+0 ] << " " << Rs_[9*i+1] << " " << Rs_[9*i+2] <<" " << Rs_[9*i+3]<<" " << Rs_[9*i+4]<<" " << Rs_[9*i+5]<<" " << Rs_[9*i+6]<<" " << Rs_[9*i+7]<<" " << Rs_[9*i+8]<< std::endl;

		}
*/
		ts_=new double[3*numViews_];
		for(int i=0;i<numViews_;i++){
			for(int j=0;j<3;j++)
				fscanfOrDie(fptr,"%lf",ts_ + 3*i + j);
		}
/*		for(int i=0;i<numViews_;i++){
			std::cout << ts_[3*i+0 ] << " " << ts_[3*i+1] << " " <<ts_[3*i+2] << std::endl;

		}
*/
		// Read 2D points
		xs_ =new double[2*numPoints_*numViews_];
		for(int i=0;i<numViews_;i++){
			for(int j=0;j<numPoints_;j++)
				for(int k=0;k<2;k++)
					fscanfOrDie(fptr,"%lf",xs_ + i*2*numPoints_ + 2*j + k);
		}
/*
		for(int i=0;i<numViews_;i++){
			for(int j=0;j<numPoints_;j++){
				for(int k=0;k<2;k++){
					std::cout << xs_[i*2*numPoints_ + 2*j + k] << " ";
				}
				std::cout<<std::endl;
			}
			std::cout<<std::endl;
		}*/


		return true;
	}






private:

	// Helper function to read in one value to a text file
	template <typename T>
	void fscanfOrDie(FILE *fptr, const char *format, T *value){
		int numScanned = fscanf(fptr, format, value);
		if(numScanned != 1){
			LOG(FATAL) << "Invalid data file";
		}
	}

	// Num of views
	int numViews_;
	// Num of points
	int numPoints_;
	// Variable that stores the K ,camera intrinsics
	double *K_;
	// Variable that stores the rotation and translation
	double *Rs_;
	double *ts_;
	// Storing 3D and 2D points.
	double *Xs_;
	double *xs_;
};
