/*
 * Pose adjustmer using keypoint likelihoods from a single image
 * Author: Krishna Murthy
*/

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>

#include <ceres/loss_function.h>
#include <ceres/iteration_callback.h>
#include <ceres/rotation.h>

// Contains definitions of various problem structs
#include "problemStructs.hpp"
// Contains various cost function struct specifications
#include "costFunctions.hpp"


int main(int argc, char** argv){

	google::InitGoogleLogging(argv[0]);

	const char *cubeFileName;
	const char *referenceCubeFileName;
	const char *outputFileName;
	const char *inputFileName;

	ceres::Solver::Options options;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	//options.linear_solver_type = ceres::DENSE_SCHUR;
	options.preconditioner_type = ceres::JACOBI;
	options.minimizer_progress_to_stdout = true;
	options.max_num_iterations = 200;


	

  	// Adjust the shape, doing it only for the left side.

	inputFileName ="/home/sarthak/Documents/adjustmentAnnotaions/data/left.txt";
	BundleAdjustmentProblem myProblem;
	if(!myProblem.loadFile(inputFileName)){
		std::cerr << "ERROR: Unable to open file " << inputFileName << std::endl;
		return 1;
	}


	std::ofstream outputFile;
	outputFileName = "/home/sarthak/Documents/adjustmentAnnotaions/adjusted_l.txt";
	outputFile.open(outputFileName);

	// Number of views
	int numViews;
	// Number of Points
	int numPoints;
	// Camera intrinsic matrix
	double *K;
	// Rotation Matrix
	double *Rs;
	// Translations
	double *ts;
	// 3D points
	double *Xs;
	// 2D points
	double *xs;
	

	numViews = myProblem.getNumViews();
	numPoints = myProblem.getNumPoints();
	K = myProblem.getK();

	Xs = myProblem.get3DPoints();
	Rs = myProblem.getRs();
	ts = myProblem.getts();
	xs = myProblem.get2DPoints();
	
	// Axis angle rotations.
	double *omegas;
	omegas = new double[3*numViews];
	for(int i=0;i<numViews;i++){
		// Expects in column major order
		ceres::RotationMatrixToAngleAxis(Rs + 9*i, omegas + 3*i);
	}

/*	// TO verify the transformation to apply on the world points to get htem in the camera coordinate frame.
	for(int i=0;i<numViews;i++){
		
		for(int j=0;j<numPoints;j++){

			// R and T given are for the camera. Hence the transformation of the world would be inverse of that transformation.
			// R was given as ROW major hence transpose not needed.
			double x[3];
			x[0] = Xs[3*j +0];
			x[1] = Xs[3*j +1];
			x[2] = Xs[3*j +2];

			x[0]-= ts[3*i + 0];
			x[1]-= ts[3*i + 1];
			x[2]-= ts[3*i + 2];

			ceres::AngleAxisRotatePoint(omegas + 3*i,x,x);
	
			double x_2d[2];
			x_2d[0] =   ( K[0]*x[0] + K[1]*x[1] + K[2]*x[2] ) / ( K[6]*x[0] + K[7]*x[1] + K[8]*x[2] )  ;
			x_2d[1] =   ( K[3]*x[0] + K[4]*x[1] + K[5]*x[2] ) / ( K[6]*x[0] + K[7]*x[1] + K[8]*x[2] ) ;

			std::cout<< x_2d[0] << " " <<x_2d[1] << std::endl;
	
		}
	}

*/


	// Declare a ceres problem.
	ceres::Problem problem;

	// Add the Adjustment cost.
	for(int i=0;i<numViews;i++){
		for(int j=0;j<numPoints;j++){
			// Add reprojection Error residual block
			ceres::CostFunction *reprojectionError = new ceres::AutoDiffCostFunction<ReprojectionError,2,3,3,3>
			(new ReprojectionError(K, xs + 2*numPoints*i + 2*j));
			
			problem.AddResidualBlock(reprojectionError,NULL, Xs+3*j, omegas + 3*i, ts + 3*i );
			//problem.AddResidualBlock(reprojectionError,new  ceres::CauchyLoss(100.0), Xs+3*j, omegas + 3*i, ts + 3*i );
			

		}
		problem.SetParameterBlockConstant(omegas + 3*i);
		problem.SetParameterBlockConstant(ts + 3*i);

	}

	// All the points are roughly planar.

	/*for(int j=0;j<numPoints;j++){
			// Add planarity error

			double n[4] = {0,0.3420,-0.9397,1.17};
			ceres::CostFunction *planarityError = new ceres::AutoDiffCostFunction<PlanarityError,1,3>
			(new PlanarityError(n));
			
			problem.AddResidualBlock(planarityError,NULL,Xs+3*j);
	}*/



	ceres::Solver::Summary summary;
  	ceres::Solve(options, &problem, &summary);
  	std::cout << summary.FullReport()<<std::endl;


  	// Write ouput to a text file.
  	for(int i=0;i<3*numPoints;){
  		outputFile << Xs[i] << " " << Xs[i+1] << " " << Xs[i+2] << std::endl;
  		i=i+3;
  	}
  	outputFile.close();


   	return 0;

}
