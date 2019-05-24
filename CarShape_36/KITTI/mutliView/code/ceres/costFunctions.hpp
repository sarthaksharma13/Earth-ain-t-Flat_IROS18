#include <ceres/ceres.h>
#include <ceres/rotation.h>
# include <math.h>

// Cost function to optimize over R,t, given the 3D and 2D keypoints, as in PnP
// X: mean shape keypoints in 3D, x: 2D observations.
struct PnPError{

	// Constructor
	PnPError(double *X, double *x, int numVec, double *v, double *K, double w, double *l) \
		: X_(X), x_(x), numVec_(numVec), v_(v), K_(K), w_(w), l_(l) {}

	// Operator method. Evaluates the cost function and computes Jacobians.
	template <typename T>
	bool operator() (const T* const rot, const T* trans, T* residuals) const {

		// Temporary variable to hold the 3D keypoint
		T P_[3];

		// Initialize the 3D point
		P_[0] = T(X_[0]);P_[1] = T(X_[1]); P_[2] = T(X_[2]);
		for(int i=0;i<numVec_;i++){
			P_[0] = P_[0] + T(l_[i ])*T(v_[3*i+0]);
			P_[1] = P_[1] + T(l_[i ])*T(v_[3*i+1]);
			P_[2] = P_[2] + T(l_[i ])*T(v_[3*i+2]);

		}

		// Rotate the point (and store the result in the same variable)
		// Order of arguments passed: (axis-angle rotation vector (size 3), point (size 3), array where result is to be stored (size 3))
		ceres::AngleAxisRotatePoint(rot, P_, P_);
		// Add the translation
		P_[0] = T(P_[0]) + trans[0];
		P_[1] = T(P_[1]) + trans[1];
		P_[2] = T(P_[2]) + trans[2];

		// Project the obtained 3D point down to the image, using the intrinsics (K)
		T p_[3];
		p_[0] = T(K_[0])*P_[0] + T(K_[1])*P_[1] + T(K_[2])*P_[2];
		p_[1] = T(K_[3])*P_[0] + T(K_[4])*P_[1] + T(K_[5])*P_[2];
		p_[2] = T(K_[6])*P_[0] + T(K_[7])*P_[1] + T(K_[8])*P_[2];

		T px_ = p_[0] / p_[2];
		T py_ = p_[1] / p_[2];

		// Compute the residuals (this one works well)
		residuals[0] = T(1)*sqrt(T(w_))*(px_ - T(x_[0]));
		residuals[1] = T(1)*sqrt(T(w_))*(py_ - T(x_[1]));
		//std::cout<<"reproj resid: " << residuals[0]+residuals[1] << " \n";
		return true;
	}

	// 3D points
	double *X_;
	// 2D observations (keypoints)
	double *x_;
	// Number of vectors
	int numVec_;
	// Top 5 Eigenvectors of the 'current' 3D point
	double *v_;
	// Intrinsic camera matrix
	double *K_;
	// Weight for the current observation
	double w_;
	// Weights for each shape basis vector (lambdas)
	double *l_;

};

// FOR GROUND PLANE POINTS ONLY
struct ReprojectionError{

	// Constructor
	ReprojectionError(double *K, double* x) : K_(K),x_(x) {}

	// The operator method. Evaluates the cost function and computes the jacobians.
	template <typename T>
	bool operator() (const T* const X , const T* const rot ,const T* t, T* residuals) const {

		// R and T given are of the camera. Hence we apply inverse of that transform to get the world in the camera frames. 
		// R was given as ROW major hence transpose not needed.
		
		T X_[3];
		X_[0] = ( X[0]-T(t[0]) );
		X_[1] = ( X[1]-T(t[1]) );
		X_[2] = ( X[2]-T(t[2]) );

		ceres::AngleAxisRotatePoint(rot, X_, X_);

		//residuals[0] = ( (T(K_[0])*X_[0] + T(K_[1])*X_[1] + T(K_[2])*X_[2] ) / (T(K_[6])*X_[0] + T(K_[7])*X_[1] + T(K_[8])*X_[2]) ) - T(x_[0] );
		//residuals[1] = ( (T(K_[3])*X_[0] + T(K_[4])*X_[1] + T(K_[5])*X_[2] ) / (T(K_[6])*X_[0] + T(K_[7])*X_[1] + T(K_[8])*X_[2]) ) - T(x_[1]);

		residuals[0] =  (T(K_[0])*X_[0] + T(K_[1])*X_[1] + T(K_[2])*X_[2] ) / (T(K_[6])*X_[0] + T(K_[7])*X_[1] + T(K_[8])*X_[2]) ;
		residuals[1] =  (T(K_[3])*X_[0] + T(K_[4])*X_[1] + T(K_[5])*X_[2] ) / (T(K_[6])*X_[0] + T(K_[7])*X_[1] + T(K_[8])*X_[2]) ;


		residuals[0] = T(1)*(residuals[0] - T(x_[0]));
		residuals[1] = T(1)*(residuals[1] - T(x_[1]));
	
		//std::cout<<"reproj resid: " << residuals[0]+residuals[1] << " \n";
	
		return true;
	}

	// Camera intrinsic matrix
	double *K_;
	// 2D image point
	double *x_;

};
// Constraint to couple the road and car in terms of normal and d and 3D
// Parameters: groundpoints, car translation
// Xw : 

struct CarGroundCouplingConstraint{

	CarGroundCouplingConstraint(double weight) : weight_(weight){}
	
	template<typename T>
	bool operator () (const T* const Xg, const T* const rotc, const T* const tc, T* residuals) const {
	
		T tcOnGround[3];
		T Nc[3];
	
		// offset from car center to ground
		T P_[3];
		P_[0] = T(0.0);
		P_[1] = T(0.9804);  // 7604 // this should half c
		P_[2] = T(0.0);
		
		T carYAxis[3];
		carYAxis[0] = T(0);
		carYAxis[1] = T(1);
		carYAxis[2] = T(0);
		
		// add offset to translation to the center of the car
		ceres::AngleAxisRotatePoint(rotc, P_, P_);

		tcOnGround[0] = tc[0] + P_[0];
		tcOnGround[1] = tc[1] + P_[1];
		tcOnGround[2] = tc[2] + P_[2];
		
		
		// rotate the car's Y-axis to get the normal to the car wheel plane. Only valid for pose estimation		
		ceres::AngleAxisRotatePoint(rotc, carYAxis, Nc);
		
		residuals[0] = T(weight_) * (ceres::DotProduct(Nc,Xg) - ceres::DotProduct(Nc,tcOnGround));
		//std::cout<<"coupling resid: " << residuals[0] << " \n";
		return true;
	}
		
	// weight of the cost function
	double weight_;
};

// optimizes for D also
// try not optimizing car R also but make sure t is optimized
struct PlaneInducedHomographyReproj{

	PlaneInducedHomographyReproj(double *K, double* x) : K_(K),x_(x) {}
	
	template<typename T>
	bool operator () (const T* const Xg, const T* const rotc, const T* const rotg, const T* const tg, const T* const d, T* residuals) const {
	
		T Nc[3];
		T r[9];
		T rg[9];			    
        T X[3];
  			
		ceres::AngleAxisToRotationMatrix(rotg, rg);	


		T carYAxis[3];
		carYAxis[0] = T(0);
		carYAxis[1] = T(1);
		carYAxis[2] = T(0);
				
		// rotate the car's Y-axis to get the normal to the car wheel plane. Only valid for pose estimation		
		ceres::AngleAxisRotatePoint(rotc, carYAxis, Nc);


		// compute reprojection based on plane induced homography
		// H = R'(I - (t*n')/d) = R'*M  ; M = [m0 m1 m2; m3 m4 m5; m6 m7 m8]
		// H = [h0 h1 h2; h3 h4 h5; h6 h7 h8]

		T M[9];
		M[0] = T(1.0) - tg[0]*Nc[0]/d[0];  
		M[1] = T(0.0) - tg[0]*Nc[1]/d[0];
		M[2] = T(0.0) - tg[0]*Nc[2]/d[0]; 

		M[3] = T(0.0) - tg[1]*Nc[0]/d[0];  
		M[4] = T(1.0) - tg[1]*Nc[1]/d[0];
		M[5] = T(0.0) - tg[1]*Nc[2]/d[0]; 

		M[6] = T(0.0) - tg[2]*Nc[0]/d[0];  
		M[7] = T(0.0) - tg[2]*Nc[1]/d[0];
		M[8] = T(1.0) - tg[2]*Nc[2]/d[0]; 
		
		T H[9];
		H[0] = rg[0]*M[0] + rg[3]*M[3] + rg[6]*M[6];
		H[1] = rg[0]*M[1] + rg[3]*M[4] + rg[6]*M[7];
		H[2] = rg[0]*M[2] + rg[3]*M[5] + rg[6]*M[8];			

		H[3] = rg[1]*M[0] + rg[4]*M[3] + rg[7]*M[6];
		H[4] = rg[1]*M[1] + rg[4]*M[4] + rg[7]*M[7];
		H[5] = rg[1]*M[2] + rg[4]*M[5] + rg[7]*M[8];		
 
		H[6] = rg[2]*M[0] + rg[5]*M[3] + rg[8]*M[6];
		H[7] = rg[2]*M[1] + rg[5]*M[4] + rg[8]*M[7];
		H[8] = rg[2]*M[2] + rg[5]*M[5] + rg[8]*M[8];		

		T X_[3];		
		X_[0] = H[0]*Xg[0] + H[1]*Xg[1] + H[2]*Xg[2];
		X_[1] = H[3]*Xg[0] + H[4]*Xg[1] + H[5]*Xg[2];		
		X_[2] = H[6]*Xg[0] + H[7]*Xg[1] + H[8]*Xg[2];

		
        residuals[0] =  (T(K_[0])*X_[0] + T(K_[1])*X_[1] + T(K_[2])*X_[2] ) / (T(K_[6])*X_[0] + T(K_[7])*X_[1] + T(K_[8])*X_[2]) ;
		residuals[1] =  (T(K_[3])*X_[0] + T(K_[4])*X_[1] + T(K_[5])*X_[2] ) / (T(K_[6])*X_[0] + T(K_[7])*X_[1] + T(K_[8])*X_[2]) ;


		residuals[0] = T(1)*(residuals[0] - T(x_[0]));
		residuals[1] = T(1)*(residuals[1] - T(x_[1]));
				
		//std::cout<<residuals[0]<<std::endl;
		return true;
	}
		
	// Camera intrinsic matrix
	double *K_;
	// 2D image point
	double *x_;
};


// optimizes for D also
// try not optimizing car R also but make sure t is optimized
struct PlaneInducedHomographyReprojC{

	PlaneInducedHomographyReprojC(double *K, double* x, double d) : K_(K),x_(x), d_(d) {}
	
	template<typename T>
	bool operator () (const T* const Xg, const T* const rotc, const T* const rotg, const T* const tg, T* residuals) const {
	
		T Nc[3];
		T r[9];
		T rg[9];			    
        T X[3];
  			
		ceres::AngleAxisToRotationMatrix(rotg, rg);	


		T carYAxis[3];
		carYAxis[0] = T(0);
		carYAxis[1] = T(1);
		carYAxis[2] = T(0);
				
		// rotate the car's Y-axis to get the normal to the car wheel plane. Only valid for pose estimation		
		ceres::AngleAxisRotatePoint(rotc, carYAxis, Nc);


		// compute reprojection based on plane induced homography
		// H = R'(I - (t*n')/d) = R'*M  ; M = [m0 m1 m2; m3 m4 m5; m6 m7 m8]
		// H = [h0 h1 h2; h3 h4 h5; h6 h7 h8]

		T M[9];
		M[0] = T(1.0) - tg[0]*Nc[0]/T(d_);  
		M[1] = T(0.0) - tg[0]*Nc[1]/T(d_);
		M[2] = T(0.0) - tg[0]*Nc[2]/T(d_); 

		M[3] = T(0.0) - tg[1]*Nc[0]/T(d_);  
		M[4] = T(1.0) - tg[1]*Nc[1]/T(d_);
		M[5] = T(0.0) - tg[1]*Nc[2]/T(d_); 

		M[6] = T(0.0) - tg[2]*Nc[0]/T(d_);  
		M[7] = T(0.0) - tg[2]*Nc[1]/T(d_);
		M[8] = T(1.0) - tg[2]*Nc[2]/T(d_); 
		
		T H[9];
		H[0] = rg[0]*M[0] + rg[3]*M[3] + rg[6]*M[6];
		H[1] = rg[0]*M[1] + rg[3]*M[4] + rg[6]*M[7];
		H[2] = rg[0]*M[2] + rg[3]*M[5] + rg[6]*M[8];			

		H[3] = rg[1]*M[0] + rg[4]*M[3] + rg[7]*M[6];
		H[4] = rg[1]*M[1] + rg[4]*M[4] + rg[7]*M[7];
		H[5] = rg[1]*M[2] + rg[4]*M[5] + rg[7]*M[8];		
 
		H[6] = rg[2]*M[0] + rg[5]*M[3] + rg[8]*M[6];
		H[7] = rg[2]*M[1] + rg[5]*M[4] + rg[8]*M[7];
		H[8] = rg[2]*M[2] + rg[5]*M[5] + rg[8]*M[8];		

		T X_[3];		
		X_[0] = H[0]*Xg[0] + H[1]*Xg[1] + H[2]*Xg[2];
		X_[1] = H[3]*Xg[0] + H[4]*Xg[1] + H[5]*Xg[2];		
		X_[2] = H[6]*Xg[0] + H[7]*Xg[1] + H[8]*Xg[2];

		
        residuals[0] =  (T(K_[0])*X_[0] + T(K_[1])*X_[1] + T(K_[2])*X_[2] ) / (T(K_[6])*X_[0] + T(K_[7])*X_[1] + T(K_[8])*X_[2]) ;
		residuals[1] =  (T(K_[3])*X_[0] + T(K_[4])*X_[1] + T(K_[5])*X_[2] ) / (T(K_[6])*X_[0] + T(K_[7])*X_[1] + T(K_[8])*X_[2]) ;


		residuals[0] = T(1)*(residuals[0] - T(x_[0]));
		residuals[1] = T(1)*(residuals[1] - T(x_[1]));
				
		//std::cout<<residuals[0]<<std::endl;
		return true;
	}
		
	// Camera intrinsic matrix
	double *K_;
	// 2D image point
	double *x_;
	double d_;
};



// The normal allignment error accounts for the fact that the plane formed by the four wheel centers is parallel to the ground plane
struct NormalAllignmentError{

	// Constructor
	//NormalAllignmentError(double *X, int numVec, double* v, double* l, double* trans) : X_(X), numVec_(numVec), v_(v), l_(l), trans_(trans) {}
	NormalAllignmentError(double *X, int numVec, double* v, double* l, double* trans, double* plane) : X_(X), numVec_(numVec), v_(v), l_(l), trans_(trans), plane_(plane) {}
	
	template <typename T>
	//bool operator() (const T* const rot,const T* plane, T* residuals) const{
	bool operator() (const T* const rot, T* residuals) const{

		// First form the four wheel centers
		T wheels_[3*4];
		for(int i=0;i<4;i++){

			T temp_[3];
			temp_[0] = T(X_[3*i + 0]);
			temp_[1] = T(X_[3*i + 1]);
			temp_[2] = T(X_[3*i + 2]);

			for(int j=0;j<numVec_;j++){

				temp_[0] += T(l_[j])*T(v_[3*numVec_*i + 3*j + 0]);
				temp_[1] += T(l_[j])*T(v_[3*numVec_*i + 3*j + 1]);
				temp_[2] += T(l_[j])*T(v_[3*numVec_*i + 3*j + 2]);
			}



			// Rotate and translate the wireframe
			ceres::AngleAxisRotatePoint(rot, temp_, temp_);
			// Add the translation
			temp_[0] = T(temp_[0]) + trans_[0];
			temp_[1] = T(temp_[1]) + trans_[1];
			temp_[2] = T(temp_[2]) + trans_[2];

			// Storing the rotated and translated wheel center.

			wheels_[3*i + 0] = temp_[0];
			wheels_[3*i + 1] = temp_[1];
			wheels_[3*i + 2] = temp_[2];
	
		}

		// First form the (any)two vectors that lie on the ground plane formed by the wheel centers.
		// Here taking left front wheel as the pivot.
		T lf_[3]; T lb_[3]; T rf_[3];
		// Left front
		lf_[0] = wheels_[0];
		lf_[1] = wheels_[1];
		lf_[2] = wheels_[2];
		// Left back
		lb_[0] = wheels_[3*1 + 0]; 
		lb_[1] = wheels_[3*1 +1]; 
		lb_[2] = wheels_[3*1 + 2];
		// Right front
		rf_[0] = wheels_[3*2 + 0];
		rf_[1] = wheels_[3*2 + 1];
		rf_[2] = wheels_[3*2 + 2];

		// Form the two vectors for cross product
		T u[3]; T v[3];
		u[0] = lf_[0] - lb_[0]; 
		u[1] = lf_[1] - lb_[1]; 
		u[2] = lf_[2] - lb_[2];

		v[0] = lf_[0] - rf_[0];
		v[1] = lf_[1] - rf_[1];
		v[2] = lf_[2] - rf_[2];


		// Calculate the normal to the wheel center plane. v1 X v2
		T wcn_[3];
		wcn_[0] = u[1]*v[2] - v[1]*u[2];
		wcn_[1] = T(-1)*( u[0]*v[2] - u[2]*v[0] );
		wcn_[2] = u[0]*v[1] - u[1]*v[0];

		// The normal direction is dependent on the pose of the car. Take care of that. Make normal point downwards
		if( wcn_[1] < T(0)){
			// Flip the vector
			wcn_[0] = T(-1.0) * wcn_[0];
			wcn_[1] = T(-1.0) * wcn_[1];
			wcn_[2] = T(-1.0) * wcn_[2];
		}


		// Calculate the cross product.
		/*residuals[0] = T(10)*wcn_[1]*T(plane[2]) - wcn_[2]*T(plane[1]);
		residuals[1] = T(10)*T(-1.0)*( wcn_[0]*T(plane[2]) - wcn_[2]*T(plane[0]));
		residuals[2] = T(10)*wcn_[0]*T(plane[1]) - wcn_[1]*T(plane[0]);
*/		
		residuals[0] = T(10)*wcn_[1]*T(plane_[2]) - wcn_[2]*T(plane_[1]);
		residuals[1] = T(10)*T(-1.0)*( wcn_[0]*T(plane_[2]) - wcn_[2]*T(plane_[0]));
		residuals[2] = T(10)*wcn_[0]*T(plane_[1]) - wcn_[1]*T(plane_[0]);


		return true;


	}

	
	// 3D points : the four wheel centers - lf,lb,rf,rb
	double *X_;
	// Read the number of vectors
	int numVec_;
	// Top numVec Eigenvectors of the wheel centers in the same order
	double *v_;
	// Weights for each shape basis vector (lambdas)
	double *l_;
	// Translation
	double* trans_;
	// Take the plane normal
	double* plane_;



};

// Error function to constrain the car translation to be on the ground plane
struct CarOnGroundError{

	// Constructor
	CarOnGroundError(double *N, double *d,double *rot) : N_(N), d_(d), rot_(rot) {}

	// The operator method. Evaluates the cost function and computes the jacobians.
	template <typename T>
	bool operator() (const T* const trans, T* residuals) const {

		T P_[3];
		P_[0] = T(0.0);
		P_[1] = T(0.9604);
		P_[2] = T(0.0);

		T tempRot[3];
		tempRot[0] = T(rot_[0]);
		tempRot[1] = T(rot_[1]);
		tempRot[2] = T(rot_[2]);


		ceres::AngleAxisRotatePoint(tempRot, P_, P_);

		T X[3];
		X[0] = P_[0] + trans[0];
		X[1] = P_[1] + trans[1];
		X[2] = P_[2] + trans[2];

		residuals[0] =  T(15)*(T(T(N_[0])*X[0]) + T(T(N_[1])*X[1]) + T(T(N_[2])*X[2]) - T(d_[0]) ) ;
		return true;
	}

	// Plane normal
	double *N_;
	// Plane distance
	double *d_;
	// Rotation of car
	double *rot_;
	
};






// Cost functino to ensure NX = D . Here all are the parameters
struct PlaneFittingError{

	PlaneFittingError(){}
	
	template<typename T>
	bool operator () (const T* const X, const T* const N, const T* const d, T* residuals) const {
	
		//residuals[0] = ((T(N[0])*T(X[0]) + T(N[1])*T(X[1]) + T(N[2])*T(X[2])) - T(d[0]));
		residuals[0] = T(0.1)*(N[0]*X[0] + N[1]*X[1] + N[2]*X[2]- d[0]);
		return true;
	}
};

// Instead of the standard Planarity error, here we use the adjusted points via B.A to adjust the car normal.
struct PlanarityError{

	// Constructor
	PlanarityError(double *X) : X_(X){}

	// The operator method. Evaluates the cost function and computes the jacobians.
	template <typename T>
	bool operator() (const T* const N,const T* const d, T* residuals) const {
		residuals[0] = T(10)*(N[0]*T(X_[0]) + N[1]*T(X_[1]) + N[2]*T(X_[2]) - d[0]);

		return true;

	}
	// Plane points
	double *X_;

	
};




// Cost function to store reprojection error resulting from the values of lambdas
struct LambdaReprojectionError{

	// Constructor
	LambdaReprojectionError(double *X, double *x, int numVec, double *v, double *K, double w, double *trans) \
		: X_(X), x_(x), numVec_(numVec), v_(v), K_(K), w_(w), trans_(trans) {}

	// Operator method. Evaluates the cost function and computes Jacobians.
	template <typename T>
	bool operator() (const T* const rot_, const T* const l, T* residuals) const {

		// 3D wireframe (before applying rotation and translation)
		T P_[3];
		P_[0] = T(X_[0]);P_[1] = T(X_[1]); P_[2] = T(X_[2]);
		// Initialize the 3D point
		for(int i=0;i<numVec_;i++){
			P_[0] = P_[0] + T(l[i])*T(v_[3*i+0]);
			P_[1] = P_[1] + T(l[i])*T(v_[3*i+1]);
			P_[2] = P_[2] + T(l[i])*T(v_[3*i+2]);

		}

		// Apply the rotation and translation
		ceres::AngleAxisRotatePoint(rot_, P_, P_);
		P_[0] += T(trans_[0]);
		P_[1] += T(trans_[1]);
		P_[2] += T(trans_[2]);

		// Project the obtained 3D point down to the image, using the intrinsics (K)
		T p_[3];
		p_[0] = T(K_[0])*P_[0] + T(K_[1])*P_[1] + T(K_[2])*P_[2];
		p_[1] = T(K_[3])*P_[0] + T(K_[4])*P_[1] + T(K_[5])*P_[2];
		p_[2] = T(K_[6])*P_[0] + T(K_[7])*P_[1] + T(K_[8])*P_[2];
		
		T px_ = p_[0] / p_[2];
		T py_ = p_[1] / p_[2];

		// Compute the residuals (this one works well)
		residuals[0] = sqrt(T(w_))*(px_ - T(x_[0]));
		residuals[1] = sqrt(T(w_))*(py_ - T(x_[1]));

		return true;
	}

	// 3D points
	double *X_;
	// 2D observations (keypoints)
	double *x_;
	// Read the number of eigen vectors
	int numVec_;
	// Top Eigenvectors
	double *v_;
	// Intrinsic camera matrix
	double *K_;
	// Weight for the current observation
	double w_;
	// Translation estimate (after PnP)
	double *trans_;


};

/*// normlas parallel after fitting a plane to gnd plane
struct CarPlaneNormalParallel{

	CarPlaneNormalParallel(const double* Xg ) {
		
		cloud->width  = 231;
		cloud->height = 1;
		cloud->points.resize (cloud->width * cloud->height);		
		Xg_ = Xg;
		
		// 
	}
	
	template<typename T>
	bool operator () () const{
	
	
	}
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
};
*/

/*********************************************    REGULARIZERS   ****************************************************/

// Constraining the vector norm to some value. (we are interested in ||Vec|| = 1)
struct NormMagnitueError{

	NormMagnitueError(double mag) : mag_(mag){}
	template <typename T>
	bool operator() (const T* vec, T* residuals) const{

		residuals[0] = T(100)*(( vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2] )- T(1.0));

		return true;

	}
	// Magnitude required : one dimension
	double mag_;


};

struct NormalRegularizer{

	NormalRegularizer() {}
	// Operator method. Evaluates the cost function and computes the Jacobians.
	template <typename T>
	bool operator() (const T* plane_N, T* residuals) const{

		residuals[0] = T(1000.0)*plane_N[0];
		residuals[1] = T(0.0)*plane_N[1];
		residuals[2] = T(0.0)*plane_N[2];

		return true;
	}
};

// Cost function to regularize the translation estimate, to prevent a huge drift from the initial estimate
struct TranslationRegularizer{

	// Constructor
	TranslationRegularizer(double *trans_init) : trans_init_(trans_init) {}

	// Operator method. Evaluates the cost function and computes the Jacobians.
	template <typename T>
	bool operator() (const T* trans, T* residuals) const{

		residuals[0] = T(100)*(trans[0]-trans_init_[0]);
		residuals[1] = T(100)*(trans[1]-trans_init_[1]);
		residuals[2] = T(100)*(trans[2]-trans_init_[2]);

		return true;
	}

	// Initial translation estimate
	double *trans_init_;

};


 //Cost function to regularize the translation estimate, to prevent a huge drift from the initial estimate
struct ScaleRegularizer{

	// Constructor
	ScaleRegularizer() {}

	// Operator method. Evaluates the cost function and computes the Jacobians.
	template <typename T>
	bool operator() (const T* s, T* residuals) const{

		residuals[0] = T(100)*(s[0] - T(1.2));

		return true;
	}
	

};


// Cost function to regularize the rotation estimate, to align the axis of rotation with the axis that we desire.
struct RotationRegularizer{

	// Constructor
	RotationRegularizer() {}

	// Operator method. Evaluates the cost function and computes the Jacobians.
	template <typename T>
	bool operator() (const T* rot, T* residuals) const{

		residuals[0] = T(1.0)*rot[0];
		residuals[1] = T(0.0)*rot[1];
		residuals[2] = T(1000.0)*rot[2];

		return true;
	}

};

// Cost function to prevent lambdas from deforming the shape strongly
struct LambdaRegularizer{

	// Constructor
	LambdaRegularizer(int numVec, double *v) : numVec_(numVec), v_(v) {}

	// Operator method. Evaluates the cost function and computes the Jacobians.
	template <typename T>
	bool operator() (const T* l, T* residuals) const{

		residuals[0] = T(0.0); residuals[1] = T(0.0); residuals[2] = T(0.0);

		for(int i=0;i<numVec_;i++){
			residuals[0] += T(l[i])*T(v_[3*i+0]);
			residuals[1] += T(l[i])*T(v_[3*i+1]);
			residuals[2] += T(l[i])*T(v_[3*i+2]);

		}

		return true;
	}

	// Number of eigen vectors
	int numVec_;
	// Top numVec eigenvectors for the 'current' 3D point
	double *v_;

};

