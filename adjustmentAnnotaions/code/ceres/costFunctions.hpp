#include <ceres/ceres.h>
#include <ceres/rotation.h>


// Define a struct to hold the distance error. This error specifies that the solution vector
// must be close to the control (initial) vector.
struct PlanarityError{

	// Constructor
	PlanarityError(double *N) : N_(N){}

	// The operator method. Evaluates the cost function and computes the jacobians.
	template <typename T>
	bool operator() (const T* const X, T* residuals) const {
		residuals[0] =  T(0.0001)*( T(T(N_[0])*X[0]) + T(T(N_[1])*X[1]) + T(T(N_[2])*X[2]) - T(N_[3]) );
		return true;
	}

	// Plane eqaution.
	double *N_;
	
};

struct AllignmentError{

	// Constructor
	AllignmentError(){}

	// The operator method. Evaluates the cost function and computes the jacobians.
	template <typename T>
	bool operator() (const T* const Ri, const T* const Rj,const T* const Ti, const T* const Tj,const T* const X, T* residuals) const {
		T X_[3];
		// rotate this point by matrix Rij

		// compute Tij
		

		// write the residual;
		return true;
	}

		
};



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


		residuals[0] = residuals[0] - T(x_[0]);
		residuals[1] = residuals[1] - T(x_[1]);
	
	
		

		return true;
	}

	// Camera intrinsic matrix
	double *K_;
	// 2D image point
	double *x_;

};