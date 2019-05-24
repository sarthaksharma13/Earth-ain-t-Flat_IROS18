/*
 * Pose adjustmer using keypoint likelihoods from a single image
 * Author: Sarthak Sharma,Krishna Murthy
*/

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <math.h>

#include <ceres/loss_function.h>
#include <ceres/iteration_callback.h>
#include <ceres/rotation.h>

// Contains definitions of various problem structs
#include "problemStructs.hpp"
// Contains various cost function struct specifications
#include "costFunctions.hpp"


int main(int argc, char** argv){

	google::InitGoogleLogging(argv[0]);

	const char *inputFileName;
	const char *outputFileName;

	
	inputFileName = "/home/sarthak/Documents/IROS_2017/data/ceres_input_multiViewAdjuster.txt";
	outputFileName = "ceres_output_mutliViewShapeAdjuster.txt";

	// Create a 'SingleViewPoseAdjustmentProblem' instance to hold the BA problem
	MultiViewShapeandPoseAdjuster myProblem;

	// Read the problem file and store relevant information
	if(!myProblem.loadFile(inputFileName)){

		std::cerr << "ERROR: Unable to open file " << inputFileName << std::endl;
		return 1;
	}

	std::ofstream outFile;
	outFile.open(outputFileName);


	// Get number of views, number of observations, number of points.
	const int numViews = myProblem.getNumViews();
	const int numObs = myProblem.getNumObs();
	const int numPts = myProblem.getNumPts();

	// Get the length, width, and height of the car
	const double carLength = myProblem.getCarLength();
	const double carWidth = myProblem.getCarWidth();
	const double carHeight = myProblem.getCarHeight();

	// The K matrix.
	double *K = myProblem.getK();

	// Get the center of the car
	double *carCenter = myProblem.getCarCenter();

	// 2d Keypoints as well as their weights.
	double *observations = myProblem.observations();
	double *observationWeights = myProblem.observationWeights();
	// Get the mean shape
	double *X_bar = myProblem.getX_bar();

	// Get the number of eigen vector
	int numVec = myProblem.getNumVec();

	// Get the top eigenvectors of the wireframe
	double *V = myProblem.getV();
	// Get the weights of the linear combination
	double *lambdas = myProblem.getLambdas();

	// Get the rotations and translations for all the views. Rotation is currently as matrix,need to convert it to axis angle.
	double *rot = myProblem.getRotations();
	double *trans = myProblem.getTranslations();

	// Convert all the rotations to axis angle form.
	double *rotAxis= new double[3*numViews];

	for(int i=0;i<numViews;i++){
		ceres::RotationMatrixToAngleAxis(rot + 9*i, rotAxis + 3*i);
	}

	// -----------------------------------
	// Construct the Optimization Problem
	// -----------------------------------


		/*
			---------------------------
				Shape Estimation : Solving for shape, holding R and t constant.
			---------------------------
		
		*/

	// Declare a Ceres problem instance to hold cost functions for pose estimation.
	ceres::Problem shapeProblem;


	for(int i=0;i<numViews;i++){


		/*double translation[3] = {0.01, 0.01, 0.01};
		// Convert the rotation estimate to an axis-angle representation
		double rotation[3] = {0.00001,0.00001,0.00001};*/

		for(int j=0;j<numPts;j++){

			// Eigen vector of the current keypoint. The eigen vector given as input also incorporates the rotation after pose adjustment.	
			double *curEigVec = new double[numVec*3];
			for(int k = 0; k < numVec; ++k){
				curEigVec[3*k+0] = V[i*3*numVec*numPts + 3*numPts*k + 3*j + 0];
				curEigVec[3*k+1] = V[i*3*numVec*numPts + 3*numPts*k + 3*j + 1];
				curEigVec[3*k+2] = V[i*3*numVec*numPts + 3*numPts*k + 3*j + 2];
				
			}

		// Create a cost function for the lambda reprojection error term
		// i.e., std reprojection error, but instead of 3D points and R,t, we solve for lambdas (shape params)	
		ceres::CostFunction *lambdaError = new ceres::AutoDiffCostFunction<LambdaReprojectionError, 2,3,42>(
			new LambdaReprojectionError(X_bar+ i*numPts*3 + 3*j, observations + i*numPts*2 + 2*j, numVec, curEigVec, K, observationWeights[i*numPts + j],trans + 3*i ));
		// Add a residual block to the problem
		shapeProblem.AddResidualBlock(lambdaError, new ceres::HuberLoss(0.8),rotAxis + 3*i,lambdas);
	


			/*ceres::CostFunction *lambdaError = new ceres::AutoDiffCostFunction<LambdaReprojectionErrorMultiView, 2,3,5>(
			new LambdaReprojectionErrorMultiView(X_bar+ i*numPts*3 + 3*j, observations + i*numPts*2 + 2*j, curEigVec, K, observationWeights[i*numPts + j],trans + 3*i ));
		// Add a residual block to the problem
		shapeProblem.AddResidualBlock(lambdaError, new ceres::HuberLoss(20),rotAxis + 3*i,lambdas);
	*/
		}

		shapeProblem.SetParameterBlockConstant(rotAxis + 3*i);

		


		/*// Regularizer on the height of the wireframe
		ceres::CostFunction *heightRegularizer = new ceres::AutoDiffCostFunction<HeightRegularizer, 1, 3, 3, 5>(
			new HeightRegularizer(X_bar+ i*3*numObs, V + 5*3*numObs, carHeight));
		shapeProblem.AddResidualBlock(heightRegularizer, NULL, rotAxis + 3*i, trans + 3*i, lambdas);

		// Regularizer on the width of the wireframe
		ceres::CostFunction *widthRegularizer = new ceres::AutoDiffCostFunction<WidthRegularizer, 1, 3, 3, 5>(
			new WidthRegularizer(X_bar + i*3*numObs, V + 5*3*numObs, carWidth));
		shapeProblem.AddResidualBlock(widthRegularizer, NULL, rotAxis + 3*i, trans + 3*i, lambdas);

		// Regularizer on the length of the wireframe
		ceres::CostFunction *lengthRegularizer = new ceres::AutoDiffCostFunction<LengthRegularizer, 1, 3, 3, 5>(
			new LengthRegularizer(X_bar + i*3*numObs, V + 5*3*numObs, carLength));
		shapeProblem.AddResidualBlock(lengthRegularizer, NULL, rotAxis + 3*i, trans + 3*i, lambdas);
*/



		/*// Regularizer on rotation (axis of rotation must be close to the Y-axis)
		ceres::CostFunction *rotationRegularizer = new ceres::AutoDiffCostFunction<RotationRegularizer, 3, 3>(
			new RotationRegularizer());
		shapeProblem.AddResidualBlock(rotationRegularizer, NULL, rotAxis + 3*i);
*/
		
		/*// Regularizer on the translation term (to prevent large drifts from the initialization)
		ceres::CostFunction *translationRegularizer = new ceres::AutoDiffCostFunction<TranslationRegularizer, 3, 3>(
			new TranslationRegularizer(trans + 3*i));
		shapeProblem.AddResidualBlock(translationRegularizer, NULL, trans + 3*i);*/
		
		
	}// num views


	// Specify solver options
	ceres::Solver::Options shapeOptions;
	shapeOptions.linear_solver_type = ceres::DENSE_SCHUR;
	shapeOptions.preconditioner_type = ceres::JACOBI;
	shapeOptions.max_num_iterations = 100;
	//shapeOptions.use_inner_iterations = true;
	shapeOptions.minimizer_progress_to_stdout = true;
	
	// Solve the problem and print the results
	ceres::Solver::Summary shapeSummary;
	ceres::Solve(shapeOptions, &shapeProblem, &shapeSummary);




	// Compute the resultant 3D wireframe and write to the file for each view.
	for(int i=0;i<numViews;i++){
		
		/*double translation[3] = {0.01, 0.01, 0.01};
		double rotAngleAxis[3] = {0.00001,0.00001,0.00001};*/

		for(int j = 0; j < numPts; j++){
			double temp[3];
			temp[0] = X_bar[i*numPts*3 + 3*j + 0];
			temp[1] = X_bar[i*numPts*3 + 3*j + 1];
			temp[2] = X_bar[i*numPts*3 + 3*j + 2];

			for(int k = 0; k < numVec; k++){
				temp[0] += lambdas[k]*V[i*3*numVec*numPts + 3*numPts*k + 3*j + 0];
				temp[1] += lambdas[k]*V[i*3*numVec*numPts + 3*numPts*k + 3*j + 1];
				temp[2] += lambdas[k]*V[i*3*numVec*numPts + 3*numPts*k + 3*j + 2];
			}
			ceres::AngleAxisRotatePoint(rotAxis + 3*i, temp, temp);
			temp[0] += trans[3*i + 0];
			temp[1] += trans[3*i + 1];
			temp[2] += trans[3*i + 2];
			outFile << temp[0] << " " << temp[1] << " " << temp[2] << std::endl;
		}

	}

 	return 0;

}// end main








































