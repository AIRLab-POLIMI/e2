#ifndef Matrix3DRotoTraslation_h
#define Matrix3DRotoTraslation_h


#include <stdlib.h> 
#include <stdio.h>
#include "const.h"

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_permutation.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_eigen.h>


//#define DEBUG2
#include "WheelChairUtil.h"

#include "RotoTraslation3D.h"

#include "Param3DRotoTraslation.h"
#include "math.h"

class Matrix3DRotoTraslation : public RotoTraslation3D{
	gsl_matrix * matrix;

	
public:
	Matrix3DRotoTraslation();
	Matrix3DRotoTraslation(double x,double y,double z, double alpha, double beta, double gamma);

	~Matrix3DRotoTraslation();

	inline void setX(double v){gsl_matrix_set(matrix,0,3,v);};
	inline void setY(double v){gsl_matrix_set(matrix,1,3,v);};
	inline void setZ(double v){gsl_matrix_set(matrix,2,3,v);};
	
	inline void setXYZ(double x,double y,double z){setX(x);setY(y);setZ(z);};
	
	inline void setAlpha(double v){
		double a,b,c;
		getAlphaBetaGamma(&a,&b,&c);
		setAlphaBetaGamma(v,b,c);
	};
	inline void setBeta(double v){
		double a,b,c;
		getAlphaBetaGamma(&a,&b,&c);
		setAlphaBetaGamma(a,v,c);
	};
	inline void setGamma(double v){
		double a,b,c;
		getAlphaBetaGamma(&a,&b,&c);
		setAlphaBetaGamma(a,b,v);
	};

	inline void setAlphaBetaGamma(double a,double b,double c){
		double ca=cos(a);
		double cb=cos(b);
		double cc=cos(c);
		double sa=sin(a);
		double sb=sin(b);
		double sc=sin(c);

		gsl_matrix_set(matrix,0,0,ca*cb);
		gsl_matrix_set(matrix,0,1,ca*sb*sc-sa*cc);
		gsl_matrix_set(matrix,0,2,ca*sb*cc+sa*sc);
		gsl_matrix_set(matrix,1,0,sa*cb);
		gsl_matrix_set(matrix,1,1,sa*sb*sc+ca*cc);
		gsl_matrix_set(matrix,1,2,sa*sb*cc-ca*sc);
		gsl_matrix_set(matrix,2,0,-sb);		
		gsl_matrix_set(matrix,2,1,cb*sc);
		gsl_matrix_set(matrix,2,2,cb*cc);	
	}


	inline double getX(){return gsl_matrix_get(matrix,0,3);};
	inline double getY(){return gsl_matrix_get(matrix,1,3);};
	inline double getZ(){return gsl_matrix_get(matrix,2,3);};
	
	inline void getXYZ(double *x,double *y,double *z){
		*x=gsl_matrix_get(matrix,0,3);
		*y=gsl_matrix_get(matrix,1,3);
		*z=gsl_matrix_get(matrix,2,3);
	}
	
	inline double getAlpha(){double a,b,c;getAlphaBetaGamma(&a,&b,&c);return a;};
	inline double getBeta(){double a,b,c;getAlphaBetaGamma(&a,&b,&c);return b;};
	inline double getGamma(){double a,b,c;getAlphaBetaGamma(&a,&b,&c);return c;};

	inline void getAlphaBetaGamma(double *a,double *b,double *c){
		*a=atan2(gsl_matrix_get(matrix,1,0),gsl_matrix_get(matrix,0,0));
		*b=atan2(-gsl_matrix_get(matrix,2,0),sqrt(gsl_matrix_get(matrix,2,1)*gsl_matrix_get(matrix,2,1)+gsl_matrix_get(matrix,2,2)*gsl_matrix_get(matrix,2,2)));
		*c=atan2(gsl_matrix_get(matrix,2,1),gsl_matrix_get(matrix,2,2));			
	};


	inline void setValue(int i,int j,double v){
		gsl_matrix_set(matrix,i,j,v);	
	};

	inline double getValue(int i,int j){
		return gsl_matrix_get(matrix,i,j);
	};

	friend Matrix3DRotoTraslation *multMatrix3D(Matrix3DRotoTraslation *m1,Matrix3DRotoTraslation *m2,Matrix3DRotoTraslation *res=NULL);

};

#endif
