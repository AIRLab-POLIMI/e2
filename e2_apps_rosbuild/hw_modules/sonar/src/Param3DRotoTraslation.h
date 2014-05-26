#ifndef Param3DRotoTraslation_h
#define Param3DRotoTraslation_h

#include "Matrix3DRotoTraslation.h"
#include "math.h"

#include "RotoTraslation3D.h"

class Param3DRotoTraslation : public RotoTraslation3D{
	double x,y,z;
	double sa,sb,sc;
	double a,b,c;
	double ca,cb,cc;
public:
	Param3DRotoTraslation();
	Param3DRotoTraslation(double x,double y,double z, double alpha, double beta, double gamma);
	~Param3DRotoTraslation();
	
	inline void setX(double px){x=px;};
	inline void setY(double py){y=py;};
	inline void setZ(double pz){z=pz;};
	inline void setXYZ(double px,double py,double pz){x=px;y=py;z=pz;};
	inline void setAlpha(double pa){a=pa;sa=sin(pa);ca=cos(pa);};
	inline void setBeta(double pb){b=pb;sb=sin(pb);cb=cos(pb);};
	inline void setGamma(double pc){c=pc;sc=sin(pc);cc=cos(pc);};
	inline void setAlpha(double psa,double pca){sa=psa;ca=pca;a=atan2(sa,ca);};
	inline void setBeta(double psb,double pcb){sb=psb;cb=pcb;b=atan2(sb,cb);};
	inline void setGamma(double psc,double pcc){sc=psc;cc=pcc;c=atan2(sc,cc);};
	inline void setAlphaBetaGamma(double pa,double pb, double pc){a=pa;b=pb;c=pc;sa=sin(a);sb=sin(b);sc=sin(c);ca=cos(a);cb=cos(b);cc=cos(c);};

	inline double getX(){return x;};
	inline double getY(){return y;};
	inline double getZ(){return z;};
	inline void getXYZ(double *px,double *py,double *pz){*px=x;*py=y;*pz=z;};
	inline double getSinA(){return sa;};
	inline double getSinB(){return sb;};
	inline double getSinC(){return sc;};
	inline double getCosA(){return ca;};
	inline double getCosB(){return cb;};
	inline double getCosC(){return cc;};
	inline double getAlpha(){return a;};
	inline double getBeta(){return b;};
	inline double getGamma(){return c;};
	inline void getAlphaBetaGamma(double *pa,double *pb,double *pc){*pa=a;*pb=b;*pc=c;};

};

#endif
