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

	// Input and output file names
	
	const char *poseBAInput       = "/home/junaid/Research@IIITH/IROS_2018/code/CarShapePoseAdjustmentWithGroundPlane/data/ceres_input_carPoseBA.txt";
	const char *groundBAInput     = "/home/junaid/Research@IIITH/IROS_2018/code/CarShapePoseAdjustmentWithGroundPlane/data/ceres_input_groundPlaneBA.txt";

	const char *poseBAOutput      = "/home/junaid/Research@IIITH/IROS_2018/code/CarShapePoseAdjustmentWithGroundPlane/data/ceres_output_carPoseBA.txt";
	const char *groundParamOutput = "/home/junaid/Research@IIITH/IROS_2018/code/CarShapePoseAdjustmentWithGroundPlane/data/ceres_output_groundParameters.txt";
	
	const char *groundBAOutput    = "/home/junaid/Research@IIITH/IROS_2018/code/CarShapePoseAdjustmentWithGroundPlane/data/ceres_output_groundBA.txt";          
	const char *groundRTOutput    = "/home/junaid/Research@IIITH/IROS_2018/code/CarShapePoseAdjustmentWithGroundPlane/data/ceres_output_groundRT.txt";          
	const char *shapeBAInput      = "/home/junaid/Research@IIITH/IROS_2018/code/CarShapePoseAdjustmentWithGroundPlane/data/ceres_output_carShapeAfterPose.txt";          
     
	// Read input Files
	
	SingleViewPoseAdjustmentProblem singleViewPoseProblem;
	if(!singleViewPoseProblem.loadFile(poseBAInput)){
		std::cerr << "ERROR: Unable to open file " << poseBAInput << std::endl;
		return 1;
	}

	GroundPlaneAdjustmentProblem groundPlaneBAProblem;
	if(!groundPlaneBAProblem.loadFile(groundBAInput)){
		std::cerr << "ERROR: Unable to open file " << groundBAInput << std::endl;
		return 1;
	}

	// Get the required parameters and initiallization for single view pose adjustment

	const int numPts           = singleViewPoseProblem.getNumPts();
	double *observations       = singleViewPoseProblem.observations();
	double *observationWeights = singleViewPoseProblem.observationWeights();
	double *K                  = singleViewPoseProblem.getK();
	double *X_bar              = singleViewPoseProblem.getX_bar();				// Mean car shape
	double *carCenter          = singleViewPoseProblem.getCarCenter();	
	const double carLength     = singleViewPoseProblem.getCarLength();              // Average car dimension
	const double carWidth      = singleViewPoseProblem.getCarWidth();
	const double carHeight     = singleViewPoseProblem.getCarHeight();
	const int numVec           = singleViewPoseProblem.getNumVec();				// Num of shape basis vectors
	double *V                  = singleViewPoseProblem.getV();					// Shape basis vectors
	double *lambdas            = singleViewPoseProblem.getLambdas();                // Eigen values


	// Get the required parameters and initiallization ground plane on which the car under obs. is
	
	const int numViews  = groundPlaneBAProblem.getNumViews();
	const int numPts_gp = groundPlaneBAProblem.getNumPts();
	double *Xs          = groundPlaneBAProblem.get3DPts();
	double *xs          = groundPlaneBAProblem.get2DPts();
	double *rot_gp      = groundPlaneBAProblem.getRs(); 						// Rotation between two camera frames
	double *trans_gp    = groundPlaneBAProblem.getts(); 						// Translation between two camera frames
	double *plane       = groundPlaneBAProblem.getPlaneParameters(); 
	
	// Load the plane parameters i.e. N and D
	

	double plane_N[3];
	double plane_D[1];	

	
	plane_N[0] = plane[0];
	plane_N[1] = plane[1];
	plane_N[2] = plane[2];
	plane_D[0] = plane[3];


	std::cerr << "-----------------------------------------------------------\n";
	std::cerr << plane_D[0] <<std::endl;
	std::cerr << "-----------------------------------------------------------\n";

	double scale[1];
	scale[0] = 1.0;
	// Set up the Ceres solver options and optimization prameters

	ceres::Solver::Options options;
	// options.linear_solver_type     = ceres::DENSE_SCHUR;
	// ITERATIVE_SCHUR > DENSE_SCHUR ~= SPARSE_SCHUR
	// options.linear_solver_type     = ceres::ITERATIVE_SCHUR;
	options.linear_solver_type        = ceres::DENSE_SCHUR;
	options.preconditioner_type       = ceres::JACOBI;
	
	// ITERATIVE_SCHUR + explicit schur complement = disaster
	// options.use_explicit_schur_complement = true;
	options.use_inner_iterations             = true;
	options.minimizer_progress_to_stdout     = true;	
	
	
	// Initialize the delta rotation and actual translation estimates of the mean car
	
	double trans[3] = {0.1, 0.1, 0.1};
	trans[0] = carCenter[0];
	trans[1] = carCenter[1];
	trans[2] = carCenter[2];
	
	double rotAngleAxis[3] = {0.01,0.01, 0.01};

	
	// Instance to hold all the cost functions for optimization	
	ceres::Problem problem;		
		
	// Construct the Optimization Problem for Car -------------------------------------------------------------------------------------		




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
		problem.AddResidualBlock(pnpError, new ceres::HuberLoss(1.5), rotAngleAxis,trans);// ceres::HuberLoss(0.8) worked for most cases
		
	}


//	problem.SetParameterBlockConstant(scale);


	double allowedOffset = 3.0;
	problem.SetParameterLowerBound(trans, 0, carCenter[0] - 0.5*allowedOffset);
	problem.SetParameterUpperBound(trans, 0, carCenter[0] + 0.5*allowedOffset);
	problem.SetParameterLowerBound(trans, 1, carCenter[1] - 0.5*allowedOffset);
	problem.SetParameterUpperBound(trans, 1, carCenter[1] + 0.5*allowedOffset);
	problem.SetParameterLowerBound(trans, 2, carCenter[2] - 0.5*allowedOffset);
	problem.SetParameterUpperBound(trans, 2, carCenter[2] + 0.5*allowedOffset);




	
	
	// Construct the Optimization Problem for Ground Plane -----------------------------------------------------------------------------		

	// Read the cam rotat	ion matrices and convert them to angle axis ( Automatic inversion 
	// is taking place due to inherent transpose action)
	
	double *omegasI, *omegas;
	omegas  = new double[3*(numViews-1)];
	omegasI = new double[3] ;

	ceres::RotationMatrixToAngleAxis(rot_gp, omegasI);
	for(int i=1;i<numViews;i++){
		// Expects in column major order
		ceres::RotationMatrixToAngleAxis(rot_gp + 9*i, omegas + 3*i -3);		
	}




	std::cout<<"NUMVIEWS:"<<numViews<<std::endl;
	double trans_gpI[3] = {0, 0, 0};



	// Add ground plane reprojection error term for BA.
	for(int i=0;i<numViews;i++){
		for(int j=0;j<numPts_gp;j++){
			// Add reprojection Error residual block
			ceres::CostFunction *reprojectionError = new ceres::AutoDiffCostFunction<ReprojectionError,2,3,3,3>
				(new ReprojectionError(K, xs + 2*numPts_gp*i + 2*j));			

			if(i == 0)
				problem.AddResidualBlock(reprojectionError,new ceres::HuberLoss(0.1), Xs+3*j, omegasI, trans_gpI);
//			else if(i == 1)
//				problem.AddResidualBlock(reprojectionError,new ceres::HuberLoss(1.0), Xs+3*j, omegas + 3*i-3, trans_gp + 3*i -3);
			else 
				problem.AddResidualBlock(reprojectionError,new ceres::HuberLoss(1.0), Xs+3*j, omegas + 3*i-3, trans_gp + 3*i -3);

			//problem.SetParameterBlockConstant(Xs +3*j);
		}

		// make the rot and trans constant i.e. we are confident on the camera pose
		//problem.SetParameterBlockConstant(omegas + 3*i);
		//problem.SetParameterBlockConstant(trans_gp + 3*i);
		

	}

	problem.SetParameterBlockConstant(omegasI);
	problem.SetParameterBlockConstant(trans_gpI);
	
	

	for(int i = 0; i<numPts_gp; ++i){
	
		ceres::CostFunction *carGroundCouplingConstraint = new ceres::AutoDiffCostFunction<CarGroundCouplingConstraint, 1, 3, 3, 3>
				(new CarGroundCouplingConstraint(5)); // with decreasing weight things are getting better. Why? still to figure out.
		problem.AddResidualBlock(carGroundCouplingConstraint, new ceres::HuberLoss(0.7), Xs + 3*i, rotAngleAxis, trans);
		//problem.SetParameterBlockConstant(Xs + 3*i);
	}



	// Add plane fitting error which optimizes for X, N, and d
	
	/*for(int i = 0; i<numPts_gp; ++i){
	
		ceres::CostFunction *planeFittingError = new ceres::AutoDiffCostFunction<PlaneFittingError, 1, 3, 3, 1>
				(new PlaneFittingError());
		problem.AddResidualBlock(planeFittingError, new ceres::HuberLoss(0.8), Xs + 3*i, plane_N, plane_D);
	}*/
		
	
	// Solve the Optimization Problem ---------------------------------------------------------------------------------------------------


	// Solve the problem and print the results
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	// std::cout << summary.FullReport() << std::endl;

	
	// Write the outputs ----------------------------------------------------------------------------------------------------------------
	
	std::ofstream outFile;
	
	// Write the optimized pose of the car
	
	outFile.open(poseBAOutput);
	double rotMat[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

	// Write the optimized rotation and translation to the output file
	ceres::AngleAxisToRotationMatrix(rotAngleAxis, rotMat);

	for(int i = 0; i < 9; ++i){
		// Write out an entry of the estimated rotation matrix to file (in column-major order)
		outFile << rotMat[i] << std::endl;
	}
	
	outFile << trans[0] << std::endl;
	outFile << trans[1] << std::endl;
	outFile << trans[2] << std::endl;
	
	outFile.close();
		
	// Write the transformed (after pose adjustment) 3D keypoints of mean car for shape adjustment
	
	outFile.open(shapeBAInput);

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
	
	outFile.close();

	// Write the optimized plane parameters to the file
/*	
	outFile.open(groundParamOutput);
	outFile << plane_N[0] << " " << plane_N[1] <<" " << plane_N[2] << " "<< plane_D[0]  << std::endl;
	outFile.close();
	
	
	std::cerr << "-----------------------------------------------------------\n";
	std::cerr << plane_D[0] <<std::endl;
	std::cerr << "-----------------------------------------------------------\n";
	*/
	// Write the optimized plane 3D points to the file



	outFile.open(groundBAOutput);
	
	for(int i=0;i<3*numPts_gp;){
  		outFile << Xs[i] << " " << Xs[i+1] << " " << Xs[i+2] << std::endl;
  		i=i+3;
  	}

	outFile.close();
	
	


	outFile.open(groundRTOutput);
	
	ceres::AngleAxisToRotationMatrix(omegasI, rotMat);
	for(int j=0; j<9; ++j){		
		// Write out an entry of the estimated rotation matrix to file (in column-major order)
		outFile << rotMat[j] << std::endl;
	}
	//write rotation
	for(int i = 0; i < (numViews-1); ++i){
		// Write the optimized rotation and translation to the output file
		ceres::AngleAxisToRotationMatrix(omegas + 3*i, rotMat);
		for(int j=0; j<9; ++j){		
			// Write out an entry of the estimated rotation matrix to file (in column-major order)
			outFile << rotMat[j] << std::endl;
		}
	}

	// write trans	
	for(int i = 0; i < 3; ++i){
		// Write out an entry of the estimated rotation matrix to file (in column-major order)
		outFile << trans_gpI[i] << std::endl;
	}
	// write trans	
	for(int i = 0; i < 3*(numViews-1); ++i){
		// Write out an entry of the estimated rotation matrix to file (in column-major order)
		outFile << trans_gp[i] << std::endl;
	}
	

	//std::cerr<<"\n\nScale "<<scale[0];

 	return 0;

}



// Add a regularizer to contraint that the wheel plane of the cars should be paralled to the ground plane on which it is moving
	/*
	double *meanWheels = new double[3*4]; 
	meanWheels[0] = X_bar[16*3 + 0]; meanWheels[1]  = X_bar[16*3 + 1]; meanWheels[2]  = X_bar[16*3 + 2]; // left front
	meanWheels[3] = X_bar[17*3 + 0]; meanWheels[4]  = X_bar[17*3 + 1]; meanWheels[5]  = X_bar[17*3 + 2]; // left back
	meanWheels[6] = X_bar[34*3 + 0]; meanWheels[7]  = X_bar[34*3 + 1]; meanWheels[8]  = X_bar[34*3 + 2]; // right front
	meanWheels[9] = X_bar[35*3 + 0]; meanWheels[10] = X_bar[35*3 + 1]; meanWheels[11] = X_bar[35*3 + 2]; // right back
	*/
	/*for(int i=0;i<4;i++){
		std::cout<<meanWheels[3*i + 0] << " " << meanWheels[3*i + 1] << " " << meanWheels[3*i + 2] << std::endl;
		
	}*/

	/*
	double *vecWheels = new double[3*numVec*4];
	for(int i=0;i<4;i++){
		for(int j=0;j<numVec;j++){
	
			int pt;
	
			// Specific to the ordering of the wheel centers.
			if(i==0)pt=16;if(i==1)pt=17;if(i==2)pt=34;if(i==3)pt=35;

			vecWheels[3*numVec*i + 3*j + 0] = V[numPts*3*j + pt*3 + 0];
			vecWheels[3*numVec*i + 3*j + 1] = V[numPts*3*j + pt*3 + 1];
			vecWheels[3*numVec*i + 3*j + 2] = V[numPts*3*j + pt*3 + 2];

		}
	}

	// To enforce that the plane of the car is parallel to the ground plane
	ceres::CostFunction *normalAllignmentError = new ceres::AutoDiffCostFunction<NormalAllignmentError, 3, 3>(
		new NormalAllignmentError(meanWheels, numVec, vecWheels, lambdas, trans,plane_N));
	problem.AddResidualBlock(normalAllignmentError, NULL, rotAngleAxis);
	*/

