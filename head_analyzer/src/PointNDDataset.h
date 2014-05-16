/*
Author: Cristian Mandelli
Mail: cristianmandelli@gmail.com
Data: 16/12/2011
Library: 
*/

#include <iterator>
#include <iostream>
#include <vector>
#include <math.h>

using namespace std;

typedef enum {SAVE_AS_INT, SAVE_AS_DOUBLE, SAVE_AS_FLOAT} pointTypeSave;
typedef enum {LOAD_AS_INT, LOAD_AS_DOUBLE, LOAD_AS_FLOAT} pointTypeLoad;
typedef enum {HERMITE_INTERPOLATION} interpolationType;

/*This class rappresent a 2D point with x and y coords, a weight and a class.
and some function to access its member.
*/
class Point2DSample
{
	private:
	double x,y;
	double weight;
	char classification;
  
	public:
	//Constructors
	Point2DSample():x(0.0),y(0.0),weight(1.0),classification('U'){}
	Point2DSample(double x, double y, double weight, char classification); 
	
	//Getters
	double getX() const { return x; }
	double getY() const { return y; }
	double getDimensionN( unsigned int N); //Starting from 1
	double getDimension() const { return 2; }
	double getWeight() const { return weight; }
	char getClassification() const { return classification; }

	//Setters
	//unsigned int getSamplingId() const { return samplingId; }
	void setX(double xx) { x = xx; }
	void setY(double yy) { y = yy; }
	void setDimensionN(unsigned int N, double value); //Strating from 1
	void setWeight(double W) { weight = W; }
	void setClassification(char C) { classification = C; }
};


/*This class rappresent a 3D point with x, y and z coords, a weight and a class.
and some function to access its members.
*/
class Point3DSample
{
	private:
	double x,y,z;
	double weight;
	char classification;
  
	public:
	//Costructors
	Point3DSample():x(0.0),y(0.0),z(0.0), weight(1.0),classification('U'){}
	Point3DSample(double x, double y, double z, double weight, char classification); 
	
	//Getters
	double getX() const { return x; }
	double getY() const { return y; }
	double getZ() const { return z; }
	double getDimensionN( unsigned int N); //Starting from 1
	double getDimension() const { return 3; }
	double getWeight() const { return weight; }
	char getClassification() const { return classification; }

	//Setters
	//unsigned int getSamplingId() const { return samplingId; }
	void setX(double xx) { x = xx; }
	void setY(double yy) { y = yy; }
	void setZ(double zz) { z = zz; }
	void setDimensionN(unsigned int N, double value); //Strating from 1
	void setWeight(double W) { weight = W; }
	void setClassification(char C) { classification = C; }
};


/*This class rappresent a 4D point with x and y coords, a weight and a class.
and some function to access its member.
*/
class Point4DSample
{
	private:
	double x,y,z,j;
	double weight;
	char classification;
  
	public:
	//Costructors
	Point4DSample():x(0.0),y(0.0),z(0.0), j(0.0), weight(1.0),classification('U'){}
	Point4DSample(double x, double y, double z, double j, double weight, char classification); 
	
	//Getters
	double getX() const { return x; }
	double getY() const { return y; }
	double getZ() const { return z; }
	double getJ() const { return j; }
	double getDimensionN( unsigned int N); //Starting from 1
	double getDimension() const { return 4; }
	double getWeight() const { return weight; }
	char getClassification() const { return classification; }

	//Setters
	//unsigned int getSamplingId() const { return samplingId; }
	void setX(double xx) { x = xx; }
	void setY(double yy) { y = yy; }
	void setZ(double zz) { z = zz; }
	void setJ(double jj) { j = jj; }
	void setDimensionN(unsigned int N, double value); //Strating from 1
	void setWeight(double W) { weight = W; }
	void setClassification(char C) { classification = C; }
};


/*This class rappresent a 5D point with x and y coords, a weight and a class.
and some function to access its member.
*/
class Point5DSample
{
	private:
	double x,y,z,j,t;
	double weight;
	char classification;
  
	public:
	//Costructors
	Point5DSample():x(0.0),y(0.0),z(0.0), j(0.0), t(0.0), weight(1.0),classification('U'){}
	Point5DSample(double x, double y, double z, double j, double t, double weight, char classification); 
	
	//Getters
	double getX() const { return x; }
	double getY() const { return y; }
	double getZ() const { return z; }
	double getJ() const { return j; }
	double getT() const { return t; }
	double getDimensionN( unsigned int N); //Starting from 1
	double getDimension() const { return 5; }
	double getWeight() const { return weight; }
	char getClassification() const { return classification; }

	//Setters
	//unsigned int getSamplingId() const { return samplingId; }
	void setX(double xx) { x = xx; }
	void setY(double yy) { y = yy; }
	void setZ(double zz) { z = zz; }
	void setJ(double jj) { j = jj; }
	void setT(double tt) { t = tt; }
	void setDimensionN(unsigned int N, double value); //Strating from 1
	void setWeight(double W) { weight = W; }
	void setClassification(char C) { classification = C; }
};


/* This class rappresent an array (implemented by vector<>) of 2DPoint previously defined
and all methods required to manipulate data.*/
class Array1D2DPointDataset
{
	private:
	//Set of ColorSample
	vector<Point2DSample> samples;
	double weight;
	char classification;
	
	//Hermite polynomials value
	double h00, h01, h10, h11;
	
	//Other functions
	void hermitePolynomials(double t);
	double angularCoefficient(double p0, double p1, double p2);

	public:
	//Costructors
	Array1D2DPointDataset():weight(1.0),classification('U'){}
	Array1D2DPointDataset(double weight, char classification); 
	
	//Getters
	double getWeight() const { return weight; }
	char getClassification() const { return classification; }
	vector<Point2DSample> getSamples() { return samples; }	//get all samples into vector
	Point2DSample getSample(unsigned int N) { return samples[N]; }		//get sample by its index
	
	//Setters
	void setWeight(double W) { weight = W; }
	void setClassification (char C) { classification = C; }
	
	//Other functions
	void insert(Point2DSample);
	void eraseByClass(char c);
	void clear();
	unsigned int size();;
	vector<Point2DSample> interpolate(unsigned int nos, int interpolationType=HERMITE_INTERPOLATION);
	
	vector<Point2DSample>::iterator begin() /*const*/ { return samples.begin(); };
	vector<Point2DSample>::iterator end() /*const*/ { return samples.end(); };

	//Save and load from file
	void save(string filename, int format, bool size, bool classification);
	void load(string filename, int format, bool size); //NOT IMPLEMENTED

};


/* This class rappresent an array (implemented by vector<>) of 3DPoint previously defined
and all methods required to manipulate data.*/
class Array1D3DPointDataset
{
	private:
	//Set of ColorSample
	vector<Point3DSample> samples;
	double weight;
	char classification;

	public:
	//Costructors
	Array1D3DPointDataset():weight(1.0),classification('U'){}
	Array1D3DPointDataset(double weight, char classification); 
	
	//Getters
	double getWeight() const { return weight; }
	char getClassification() const { return classification; }
	vector<Point3DSample> getSamples() { return samples; }	//get all samples into vector
	Point3DSample getSample(unsigned int N) { return samples[N]; }		//get sample by its index
	
	//Setters
	void setWeight(double W) { weight = W; }
	void setClassification (char C) { classification = C; }
	
	//Other functions
	void insert(Point3DSample);
	void eraseByClass(char c);
	void clear();
	unsigned int size();
	vector<Point3DSample>::iterator begin() /*const*/ { return samples.begin(); };
	vector<Point3DSample>::iterator end() /*const*/ { return samples.end(); };

	//Save and load from file
	void save(string filename, int format, bool size, bool classification);
	void load(string filename, int format, bool size, bool classification);

};


/* This class rappresent an array (implemented by vector<>) of 4DPoint previously defined
and all methods required to manipulate data.*/
class Array1D4DPointDataset
{
	private:
	//Set of ColorSample
	vector<Point4DSample> samples;
	double weight;
	char classification;

	public:
	//Costructors
	Array1D4DPointDataset():weight(1.0),classification('U'){}
	Array1D4DPointDataset(double weight, char classification); 
	
	//Getters
	double getWeight() const { return weight; }
	char getClassification() const { return classification; }
	vector<Point4DSample> getSamples() { return samples; }	//get all samples into vector
	Point4DSample getSample(unsigned int N) { return samples[N]; }		//get sample by its index
	
	//Setters
	void setWeight(double W) { weight = W; }
	void setClassification (char C) { classification = C; }
	
	//Other functions
	void insert(Point4DSample);
	void eraseByClass(char c);
	void clear();
	unsigned int size();
	vector<Point4DSample>::iterator begin() /*const*/ { return samples.begin(); };
	vector<Point4DSample>::iterator end() /*const*/ { return samples.end(); };

	//Save and load from file
	void save(string filename, int format, bool size, bool classification);
	void load(string filename, int format, bool size, bool classification);

};


/* This class rappresent an array (implemented by vector<>) of 5DPoint previously defined
and all methods required to manipulate data.*/
class Array1D5DPointDataset
{
	private:
	//Set of ColorSample
	vector<Point5DSample> samples;
	double weight;
	char classification;

	public:
	//Costructors
	Array1D5DPointDataset():weight(1.0),classification('U'){}
	Array1D5DPointDataset(double weight, char classification); 
	
	//Getters
	double getWeight() const { return weight; }
	char getClassification() const { return classification; }
	vector<Point5DSample> getSamples() { return samples; }	//get all samples into vector
	Point5DSample getSample(unsigned int N) { return samples[N]; }		//get sample by its index
	
	//Setters
	void setWeight(double W) { weight = W; }
	void setClassification (char C) { classification = C; }
	
	//Other functions
	void insert(Point5DSample);
	void eraseByClass(char c);
	void clear();
	unsigned int size();
	vector<Point5DSample>::iterator begin() /*const*/ { return samples.begin(); };
	vector<Point5DSample>::iterator end() /*const*/ { return samples.end(); };

	//Save and load from file
	void save(string filename, int format, bool size, bool classification);
	void load(string filename, int format, bool size, bool classification);

};


/* This class rappresent an array (implemented by vector<>) of Array1DDataset previously defined
and all methods required to manipulate data.*/
class Array2D2DPointDataset
{
	private:
	//Set of Array1DDatasetample
	vector<Array1D2DPointDataset> arraySamples;
	double weight;
	char classification;

	public:
	//Costructors
	Array2D2DPointDataset():weight(1.0),classification('U'){}
	Array2D2DPointDataset(double weight, char classification); 
	
	//Getters
	double getWeight() const { return weight; }
	char getClassification() const { return classification; }
	vector<Array1D2DPointDataset> getArraySamples() { return arraySamples; }				//get all samples into vector
	Array1D2DPointDataset getArraySample(unsigned int N) { return arraySamples[N]; }		//get sample by its index

	//Setters
	void setWeight(double W) { weight = W; }
	void setClassification (char C) { classification = C; }
	
	//Other functions
	void insert(Array1D2DPointDataset);
	void clear();
	unsigned int size();
	vector<Array1D2DPointDataset>::iterator begin() /*const*/ { return arraySamples.begin(); };
	vector<Array1D2DPointDataset>::iterator end() /*const*/ { return arraySamples.end(); };
	
	//Save and load from file
	void save(string filename, int format, bool size, bool classification);
	void load(string filename, int format, bool size);
};

/* This class rappresent an array (implemented by vector<>) of Array1DDataset previously defined
and all methods required to manipulate data.*/
class Array2D3DPointDataset
{
	private:
	//Set of Array1DDatasetample
	vector<Array1D3DPointDataset> arraySamples;
	double weight;
	char classification;

	public:
	//Costructors
	Array2D3DPointDataset():weight(1.0),classification('U'){}
	Array2D3DPointDataset(double weight, char classification); 
	
	//Getters
	double getWeight() const { return weight; }
	char getClassification() const { return classification; }
	vector<Array1D3DPointDataset> getArraySamples() { return arraySamples; }	//get all samples into vector
	Array1D3DPointDataset getArraySample(unsigned int N) { return arraySamples[N]; }		//get sample by its index

	//Setters
	void setWeight(double W) { weight = W; }
	void setClassification (char C) { classification = C; }
	
	//Other functions
	void insert(Array1D3DPointDataset);
	void clear();
	unsigned int size();
	vector<Array1D3DPointDataset>::iterator begin() /*const*/ { return arraySamples.begin(); };
	vector<Array1D3DPointDataset>::iterator end() /*const*/ { return arraySamples.end(); };
	
	//Save and load from file
	void save(string filename, int format, bool size, bool classification);
	void load(string filename, int format, bool size);
};

/* This class rappresent an array (implemented by vector<>) of Array1DDataset previously defined
and all methods required to manipulate data.*/
class Array2D4DPointDataset
{
	private:
	//Set of Array1DDatasetample
	vector<Array1D4DPointDataset> arraySamples;
	double weight;
	char classification;

	public:
	//Costructors
	Array2D4DPointDataset():weight(1.0),classification('U'){}
	Array2D4DPointDataset(double weight, char classification); 
	
	//Getters
	double getWeight() const { return weight; }
	char getClassification() const { return classification; }
	vector<Array1D4DPointDataset> getArraySamples() { return arraySamples; }	//get all samples into vector
	Array1D4DPointDataset getArraySample(unsigned int N) { return arraySamples[N]; }		//get sample by its index

	//Setters
	void setWeight(double W) { weight = W; }
	void setClassification (char C) { classification = C; }
	
	//Other functions
	void insert(Array1D4DPointDataset);
	void clear();
	unsigned int size();
	vector<Array1D4DPointDataset>::iterator begin() /*const*/ { return arraySamples.begin(); };
	vector<Array1D4DPointDataset>::iterator end() /*const*/ { return arraySamples.end(); };
	
	//Save and load from file
	void save(string filename, int format, bool size, bool classification);
	void load(string filename, int format, bool size);
};
