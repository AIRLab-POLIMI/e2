#ifndef RotoTraslation3D_h_
#define RotoTraslation3D_h_

class RotoTraslation3D{

	public:
		virtual void setX(double v)=0;
		virtual void setY(double v)=0;
		virtual void setZ(double v)=0;
		virtual void setXYZ(double x,double y,double z)=0;
		virtual void setAlpha(double v)=0;
		virtual void setBeta(double v)=0;
		virtual void setGamma(double v)=0;
		virtual void setAlphaBetaGamma(double a,double b,double c)=0;
		

		virtual double getX()=0;
		virtual double getY()=0;
		virtual double getZ()=0;
		virtual void getXYZ(double *x,double *y,double *z)=0;
		virtual double getAlpha()=0;
		virtual double getBeta()=0;
		virtual double getGamma()=0;
		virtual void getAlphaBetaGamma(double *a,double *b,double *c)=0;

};

#endif
