/*
 * Single view car pose adjuster with two view ground plane constraints
 * Authors: Krishna Murthy 
 		  Sarthak Sharma
 		  Junaid A. Ansari 		    
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

	

	// ---------------------- 
	// Inputs 
	// ---------------------
	const char *inputFileName;
	inputFileName ="/home/sarthak/Documents/CarShape_36/synthia/data/ceres_input_singleViewPoseAdjuster.txt";
	SingleViewPoseAdjustmentProblem myProblem;
	if(!myProblem.loadFile(inputFileName)){
		std::cerr << "ERROR: Unable to open file " << inputFileName << std::endl;
		return 1;
	}

	inputFileName = "/home/sarthak/Documents/CarShape_36/synthia/data/ceres_input_groundPlane.txt";
	GroundPlaneAdjustmentProblem grProblem;
	if(!grProblem.loadFile(inputFileName)){
		std::cerr << "ERROR: Unable to open file " << inputFileName << std::endl;
		return 1;
	}

	// Car parameters
	double *plane;
	double plane_N[3]; double plane_D[1];

	const int numPts = myProblem.getNumPts();
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
	// Get the plane parameters on which the car is.
	plane = myProblem.getPlaneParameters(); 
	
	plane_N[0] = plane[0];
	plane_N[1] = plane[1];
	plane_N[2] = plane[2];
	plane_D[0] = plane[3];

/*	// Ground plane parameters
	const int numViews = grProblem.getNumViews();
	const int numPts_gp = grProblem.getNumPts();

	double *Xs = grProblem.get3DPts();
	double *xs = grProblem.get2DPts();
	double *rot_gp = grProblem.getRs(); // Of the camera over the num Views
	double *trans_gp = grProblem.getts(); // Of the camera over the num Views
	plane = grProblem.getPlaneParameters(); 
	plane_N[0] = plane[0];
	plane_N[1] = plane[1];
	plane_N[2] = plane[2];
	plane_D[0] = plane[3];


*/
	// ----------------------
	// Set up the optimization parameters
	// ----------------------

	// Specify solver options
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	options.preconditioner_type = ceres::JACOBI;
	options.use_inner_iterations = true;
	options.minimizer_progress_to_stdout =false;
	
	// Initialize the rotation and translation estimates
	double trans[3];
	trans[0] = carCenter[0];
	trans[1] = carCenter[1];
	trans[2] = carCenter[2];

	// Convert the rotation estimate to an axis-angle representation
	double rotAngleAxis[3] = {0.01, 1, 0.01};
	
	// Instance to hold all the cost functions for optimization	
	ceres::Problem problem;		


	// **CAR**
		
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
		problem.AddResidualBlock(pnpError, new ceres::HuberLoss(20), rotAngleAxis,trans);// ceres::HuberLoss(0.8) worked for most cases
		
	}



	double allowedOffset = 4.5;
	if (trans[2] < 12){
		allowedOffset = 1.5;
	}

	problem.SetParameterLowerBound(trans, 0, carCenter[0] - 0.5*allowedOffset);
	problem.SetParameterUpperBound(trans, 0, carCenter[0] + 0.5*allowedOffset);
	problem.SetParameterLowerBound(trans, 1, carCenter[1] - 0.5*allowedOffset);
	problem.SetParameterUpperBound(trans, 1, carCenter[1] + 0.5*allowedOffset);
	problem.SetParameterLowerBound(trans, 2, carCenter[2] - 0.5*allowedOffset);
	problem.SetParameterUpperBound(trans, 2, carCenter[2] + 0.5*allowedOffset);



	// Add a regularizer to contraint the plane of the cars to be paralled to the ground plane
	double *meanWheels = new double[3*4]; 
	meanWheels[0] = X_bar[2*3 + 0]; meanWheels[1] = X_bar[2*3 + 1]; meanWheels[2] = X_bar[2*3 + 2]; // left front
	meanWheels[3] = X_bar[1*3 + 0]; meanWheels[4] = X_bar[1*3 + 1]; meanWheels[5] = X_bar[1*3 + 2]; // left back
	meanWheels[6] = X_bar[20*3 + 0]; meanWheels[7] = X_bar[20*3 + 1]; meanWheels[8] = X_bar[20*3 + 2]; // right front
	meanWheels[9] = X_bar[19*3 + 0]; meanWheels[10] = X_bar[19*3 + 1]; meanWheels[11] = X_bar[19*3 + 2]; // right back

	/*for(int i=0;i<4;i++){
		std::cout<<meanWheels[3*i + 0] << " " << meanWheels[3*i + 1] << " " << meanWheels[3*i + 2] << std::endl;
		
	}*/

	double *vecWheels = new double[3*numVec*4];
	for(int i=0;i<4;i++){
		for(int j=0;j<numVec;j++){
			int pt;
			// Specific to the ordering of the wheel centers.
			if(i==0)pt=2;if(i==1)pt=1;if(i==2)pt=20;if(i==3)pt=19;

			vecWheels[3*numVec*i + 3*j + 0] = V[numPts*3*j + pt*3 + 0];
			vecWheels[3*numVec*i + 3*j + 1] = V[numPts*3*j + pt*3 + 1];
			vecWheels[3*numVec*i + 3*j + 2] = V[numPts*3*j + pt*3 + 2];

		}

	}

	// To enforce that the plane of the car is parallel to the ground plane
	ceres::CostFunction *normalAllignmentError = new ceres::AutoDiffCostFunction<NormalAllignmentError, 3, 3>(
		new NormalAllignmentError(meanWheels, numVec, vecWheels, lambdas, trans,plane_N));
	problem.AddResidualBlock(normalAllignmentError, NULL, rotAngleAxis);

	
	// Add a regularizer to the translation term (to prevent a huge drift from the initialization)	
	ceres::CostFunction *translationRegularizer = new ceres::AutoDiffCostFunction<TranslationRegularizer, 3, 3>(
		new TranslationRegularizer(carCenter));
	problem.AddResidualBlock(translationRegularizer, new ceres::HuberLoss(1), trans);
	
	// Cost function to enfore thet car translates on the ground plane 
	ceres::CostFunction * carOnGroundError = new ceres::AutoDiffCostFunction<CarOnGroundError,1,3>(
		new CarOnGroundError(plane_N, plane_D, rotAngleAxis));
	problem.AddResidualBlock(carOnGroundError,NULL,trans);




/*	// **GROUND PLANE**
	double *omegas;
	omegas = new double[3*numViews];
	for(int i=0;i<numViews;i++){
		// Expects in column major order
		ceres::RotationMatrixToAngleAxis(rot_gp + 9*i, omegas + 3*i);
	}


	// Add the Adjustment cost.
	for(int i=0;i<numViews;i++){
		for(int j=0;j<numPts_gp;j++){
			// Add reprojection Error residual block
			ceres::CostFunction *reprojectionError = new ceres::AutoDiffCostFunction<ReprojectionError,2,3,3,3>
			(new ReprojectionError(K, xs + 2*numPts_gp*i + 2*j));
			
			problem.AddResidualBlock(reprojectionError,new ceres::HuberLoss(0.8), Xs+3*j, omegas + 3*i, trans_gp + 3*i );

		}


		problem.SetParameterBlockConstant(omegas + 3*i);
		problem.SetParameterBlockConstant(trans_gp + 3*i);

	}

	// Add the Planarity Error
	for(int i=0;i<numPts;i++){
		ceres::CostFunction *planarityError = new ceres::AutoDiffCostFunction<PlanarityError,1,3,1>
			(new PlanarityError(Xs+ 3*i));
	
		problem.AddResidualBlock(planarityError,new ceres::TukeyLoss(0.7),plane_N,plane_D);			
	
	}

	// Constraint the plane normal to be unit norm.
	ceres::CostFunction *normMagnitueError = new ceres::AutoDiffCostFunction<NormMagnitueError,1,3>
			(new NormMagnitueError(1.0));
	problem.AddResidualBlock(normMagnitueError,NULL,plane_N);

	// Contraint the plane normal to have only Y and Z components : not dealing with bank in road yet.
	ceres::CostFunction *normalRegularizer = new ceres::AutoDiffCostFunction<NormalRegularizer,1,3>
			(new NormalRegularizer());
	problem.AddResidualBlock(normalRegularizer,NULL,plane_N);


*/

	// ----------------------
	// Solve the problem and print the results
	// ----------------------
	
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	// -----------------------------------
	// Write the outputs
	// -----------------------------------


	// Pose of the car
	const char *outputFileName;
	std::ofstream outFile;
	outputFileName = "/home/sarthak/Documents/CarShape_36/synthia/ceres_output_singleViewPoseAdjuster.txt";
	outFile.open(outputFileName);

	
	double rotMat[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
	// Open the output file (to write the result)
	
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
	outFile.close();
	
	

	// Shape after pose adjustment
	
	outputFileName = "/home/sarthak/Documents/CarShape_36/synthia/poseAndShapeIntermediate.txt";
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
		}

		ceres::AngleAxisRotatePoint(rotAngleAxis, temp, temp);
		temp[0] += trans[0];
		temp[1] += trans[1];
		temp[2] += trans[2];

		// Write the output to file
		outFile << temp[0] << " " << temp[1] << " " << temp[2] << std::endl;
	}
	outFile.close();

/*
	// Adjusted ground plane

	outputFileName = "/home/sarthak/Documents/CarShape_36/synthia/groundPlane.txt";
	outFile.open(outputFileName);

	for(int i=0;i<3*numPts_gp;){
  		outFile << Xs[i] << " " << Xs[i+1] << " " << Xs[i+2] << std::endl;
  		i=i+3;
  	}
  	outFile.close();

  	// Ground plane parameters
  	
	outputFileName = "/home/sarthak/Documents/CarShape_36/synthia/planeParameters.txt";
	outFile.open(outputFileName);

	outFile << plane_N[0] << " " << plane_N[1] <<" " << plane_N[2] << " "<< plane_D[0]  << std::endl;
	outFile.close();

*/
 	return 0;

}



