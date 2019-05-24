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

	

	// ---------------------- 
	// Inputs 
	// ---------------------
	const char *inputFileName;
	inputFileName ="/home/sarthak/Documents/CarShape_36/KITTI/data/ceres_input_singleViewPoseAdjuster.txt";
	SingleViewPoseAdjustmentProblem myProblem;
	if(!myProblem.loadFile(inputFileName)){
		std::cerr << "ERROR: Unable to open file " << inputFileName << std::endl;
		return 1;
	}

	// ---------------------------
	// Get the required variables
	//----------------------------

	// Car parameters
	const int numPts = myProblem.getNumPts();
	std::cout << numPts << std::endl;
	double *observations = myProblem.observations();
	double *observationWeights = myProblem.observationWeights();
	double *K = myProblem.getK();
	double *X_bar = myProblem.getX_bar();
	// Get the center of the car
	double *carCenter = myProblem.getCarCenter();
	// Get the length, width, and height of the car
	const double carLength = myProblem.getCarLength();
	const double carWidth = myProblem.getCarWidth();
	const double carHeight = myProblem.getCarHeight();
	const int numVec = myProblem.getNumVec();
	// Get the top 5 eigenvectors of the wireframe
	double *V = myProblem.getV();
	// Get the weights of the linear combination
	double *lambdas = myProblem.getLambdas();


	// ----------------------
	// Set up the optimization parameters
	// ----------------------


	// Specify solver options
	ceres::Solver::Options options;
	// options.linear_solver_type = ceres::DENSE_SCHUR;
	// ITERATIVE_SCHUR > DENSE_SCHUR ~= SPARSE_SCHUR
	// options.linear_solver_type = ceres::ITERATIVE_SCHUR;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	options.preconditioner_type = ceres::JACOBI;
	// ITERATIVE_SCHUR + explicit schur complement = disaster
	// options.use_explicit_schur_complement = true;
	options.use_inner_iterations = true;
	options.minimizer_progress_to_stdout =true;
	
	// Initialize the rotation and translation estimates
	double trans[3] = {0.1, 0.1, 0.1};
	// Convert the rotation estimate to an axis-angle representation
	double rotAngleAxis[3] = {0.01, 1, 0.01};
	//double rotAngleAxis[3] = {0.01, 1, 1.0};

	
	// -----------------------------------
	// Construct the Optimization Problem 
	// -----------------------------------



	// Declare a Ceres problem instance to hold cost functions
	ceres::Problem problem;

	// FOR CAR ==>
	// For each observation, add a standard PnP error (reprojection error) residual block
	for(int i = 0; i < numPts; ++i){
		// Create a vector of eigenvalues for the current keypoint
		double *curEigVec = new double[numVec*3];
		// std::cout << "curEigVec: ";
		for(int j = 0; j < numVec; ++j){
			curEigVec[3*j+0] = V[3*numPts*j + 3*i + 0];
			curEigVec[3*j+1] = V[3*numPts*j + 3*i + 1];
			curEigVec[3*j+2] = V[3*numPts*j + 3*i + 2];
			
		}

		// Create a cost function for the PnP error term
		ceres::CostFunction *pnpError = new ceres::AutoDiffCostFunction<PnPError, 2, 3, 3>( 
			new PnPError(X_bar+3*i, observations+2*i, numVec, curEigVec, K, observationWeights[i], lambdas));

		// Add a residual block to the problem
		problem.AddResidualBlock(pnpError, new ceres::HuberLoss(0.8), rotAngleAxis,trans);// ceres::HuberLoss(0.8) worked for most cases
	}


	// Add a regularizer to the translation term (to prevent a huge drift from the initialization)
	ceres::CostFunction *translationRegularizer = new ceres::AutoDiffCostFunction<TranslationRegularizer, 3, 3>(
		new TranslationRegularizer(carCenter));
	problem.AddResidualBlock(translationRegularizer, new ceres::HuberLoss(0.1), trans);


	// Add a rotation regularizer, to ensure that the rotation is about the Y-axis
	ceres::CostFunction *rotationRegularizer = new ceres::AutoDiffCostFunction<RotationRegularizer, 3, 3>(
		new RotationRegularizer());
	problem.AddResidualBlock(rotationRegularizer, NULL, rotAngleAxis);


	// Set lower and upper bounds on the translation estimates returned
	problem.SetParameterLowerBound(trans, 0, -1);
	problem.SetParameterUpperBound(trans, 0, 1);
	problem.SetParameterLowerBound(trans, 1, -1);
	problem.SetParameterUpperBound(trans, 1, 1);
	problem.SetParameterLowerBound(trans, 2, -1);
	problem.SetParameterUpperBound(trans, 2, 1);

	// -----------------------------------
	// Solve the Optimization Problem
	// -----------------------------------

	

	// Solve the problem and print the results
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	
	// -----------------------------------
	// Write the outputs
	// -----------------------------------


	// Pose of the car
	const char *outputFileName;
	std::ofstream outputFile;
	outputFileName = "/home/sarthak/Documents/CarShape_36/KITTI/ceres_output_singleViewPoseAdjuster.txt";
	outputFile.open(outputFileName);

	double rotMat[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
	// Open the output file (to write the result)
	std::ofstream outFile;
	outFile.open(outputFileName);

	// Write the estimated rotation and translation to the output file
	ceres::AngleAxisToRotationMatrix(rotAngleAxis, rotMat);
	for(int i = 0; i < 9; ++i){
		// Write out an entry of the estimated rotation matrix to file (in column-major order)
		outFile << rotMat[i] << std::endl;
		
	}
	// Write the translation estimate to file
	outFile << trans[0] << std::endl;
	outFile << trans[1] << std::endl;
	outFile << trans[2] << std::endl;
	
	

	// Shape after pose adjustment
	const char *outputFileName2;
	outputFileName2 = "/home/sarthak/Documents/CarShape_36/KITTI/poseAndShapeIntermediate.txt";
	std::ofstream outFile2;
	outFile2.open(outputFileName2);

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
		outFile2 << temp[0] << " " << temp[1] << " " << temp[2] << std::endl;
	}


 	return 0;

}
