/*
 * Shape adjustmer using keypoint likelihoods from a single image
 * Authors: Junaid A. Ansari
 		  Sarthak Sharma
 		  Krishna Murthy 
*/

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>

#include <ceres/loss_function.h>
#include <ceres/iteration_callback.h>
#include <ceres/rotation.h>

#include "problemStructs.hpp"
#include "costFunctions.hpp"


int main(int argc, char** argv){

	google::InitGoogleLogging(argv[0]);

	const char *cubeFileName;
	const char *referenceCubeFileName;
	const char *outputFileName;
	const char *inputFileName;

	ceres::Solver::Options options;
	//options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	options.preconditioner_type = ceres::JACOBI;
	options.minimizer_progress_to_stdout = false;
	//options.max_num_iterations = 200;



  	// Adjust the shape, doing it only for the left side.

	inputFileName ="/home/sarthak/Documents/CarShape_36/synthia/data/ceres_input_singleViewShapeAdjuster.txt";
	SingleViewShapeAdjustmentProblem myProblem;
	if(!myProblem.loadFile(inputFileName)){
		std::cerr << "ERROR: Unable to open file " << inputFileName << std::endl;
		return 1;
	}


	std::ofstream outputFile;
	outputFileName = "/home/sarthak/Documents/CarShape_36/synthia/ceres_output_singleViewShapeAdjuster.txt";
	outputFile.open(outputFileName);


	// ---------------------------
	// Get the data required
	// ---------------------------

	// Num of observations and number of points
	
	const int numPts = myProblem.getNumPts();
	// 2D predicted keypoints
	double *observations = myProblem.observations();
	// Corresponding weights
	double *observationWeights = myProblem.observationWeights();
	// Camera matrix
	double *K = myProblem.getK();
	// Mean wireframe
	double *X_bar = myProblem.getX_bar();
	// Get the center of the car
	double *carCenter = myProblem.getCarCenter();
	// Get the length, width, and height of the car
	const double carLength = myProblem.getCarLength();
	const double carWidth = myProblem.getCarWidth();
	const double carHeight = myProblem.getCarHeight();
	// Get the num of eigen vectors used.
	const int numVec = myProblem.getNumVec();
	// Get the top numvec_ eigenvectors of the wireframe
	double *V = myProblem.getV();
	// Get the weights of the linear combination
	double *lambdas = myProblem.getLambdas();
	
	
	// Get the rotation and translation estimates (after PnP)
	double *rot = myProblem.getRot();
	double *trans = myProblem.getTrans();
	// Convert the rotation estimate to an axis-angle representation
	double rotAngleAxis[3] = {0, 0.001, 0};
	ceres::RotationMatrixToAngleAxis(rot, rotAngleAxis);
	// std::cout << "rotAngleAxis: " << rotAngleAxis[0] << " " << rotAngleAxis[1] << " " << rotAngleAxis[2] << std::endl;
		
	
	// -----------------------------------
	// Construct the Optimization Problem
	// -----------------------------------

	

	// Declare a Ceres problem instance to hold cost functions
	ceres::Problem problem;

	// For each observation, add a standard PnP error (reprojection error) residual block
	for(int i = 0; i < numPts; ++i){
		// Create a vector of eigenvalues for the current keypoint
		double *curEigVec = new double[3*numVec];
		// std::cout << "curEigVec: ";
		for(int j = 0; j < numVec; ++j){
			curEigVec[3*j+0] = V[3*numPts*j + 3*i + 0];
			curEigVec[3*j+1] = V[3*numPts*j + 3*i + 1];
			curEigVec[3*j+2] = V[3*numPts*j + 3*i + 2];
			// std::cout << V[3*numObs*j + 3*i + 0] << " " << V[3*numObs*j + 3*i + 1] << " " << \
			// 	V[3*numObs*j + 3*i + 2] << std::endl;
		}

		// Create a cost function for the lambda reprojection error term
		// i.e., std reprojection error, but instead of 3D points and R,t, we solve for lambdas (shape params)
		
		ceres::CostFunction *lambdaError = new ceres::AutoDiffCostFunction<LambdaReprojectionError, 2, 3, 42 >(
			new LambdaReprojectionError(X_bar+3*i, observations+2*i, numVec, curEigVec, K, observationWeights[i], trans));

		
		// Add a residual block to the problem
		problem.AddResidualBlock(lambdaError, new ceres::HuberLoss(15), rotAngleAxis, lambdas);

		// Add a regularizer (to prevent lambdas from growing too large)
		ceres::CostFunction *lambdaRegularizer = new ceres::AutoDiffCostFunction<LambdaRegularizer, 3, 42>(
			new LambdaRegularizer(numVec,curEigVec));
		// Add a residual block to the problem
		problem.AddResidualBlock(lambdaRegularizer, new ceres::HuberLoss(10), lambdas);

		// // Create a cost function to regularize 3D keypoint locations (alignment error)
		// ceres::CostFunction *alignmentError = new ceres::AutoDiffCostFunction<LambdaAlignmentError, 3, 5>(
		// 	new LambdaAlignmentError(X_bar+3*i, observations+2*i, curEigVec, K, observationWeights[i], X_bar_initial+3*i));

	}
	// We don't want to optimize over the rotation here. We want it to remain the same as it was after PnP.
	problem.SetParameterBlockConstant(rotAngleAxis);


	ceres::Solver::Summary summary;
  	ceres::Solve(options, &problem, &summary);
  	//std::cout << summary.FullReport()<<std::endl;

  	// Open the output file (to write the result)
	std::ofstream outFile;
	outFile.open(outputFileName);


  	// Compute the resultant 3D wireframe
	for(int i = 0; i < numPts; ++i){
		double temp[3];
		temp[0] = X_bar[3*i];
		temp[1] = X_bar[3*i+1];
		temp[2] = X_bar[3*i+2];

		for(int j = 0; j < numVec; ++j){
			temp[0] += lambdas[j]*V[3*numPts*j + 3*i + 0];
			temp[1] += lambdas[j]*V[3*numPts*j + 3*i + 1];
			temp[2] += lambdas[j]*V[3*numPts*j + 3*i + 2];
			// std::cout << V[3*numObs*j + 3*i + 0] << " " << V[3*numObs*j + 3*i + 1] << " " << \
			// 	V[3*numObs*j + 3*i + 2] << std::endl;
		}

		ceres::AngleAxisRotatePoint(rotAngleAxis, temp, temp);
		temp[0] += trans[0];
		temp[1] += trans[1];
		temp[2] += trans[2];

		// Write the output to file
		outFile << temp[0] << " " << temp[1] << " " << temp[2] << std::endl;
	}


	

	const char *outputFileName2;
	outputFileName2 = "/home/sarthak/Documents/CarShape_36/synthia/lambdasAfterShape.txt";

	std::ofstream outFile2;
	outFile2.open(outputFileName2);

	for(int i=0;i<numVec;i++){
		outFile2 << lambdas[i] << " ";
	}

	
   	return 0;

}
