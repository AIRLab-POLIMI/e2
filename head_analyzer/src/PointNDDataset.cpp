/*
Author: Cristian Mandelli
Mail: cristianmandelli@gmail.com
Data: 16/012/2011
Library: 
*/

#include "PointNDDataset.h"
#include <fstream>
#include <iostream>

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//							2D-POINT
///////////////////////////////////////////////////////////////////////////////
Point2DSample::Point2DSample(double xx, double yy, double ww, char cc)
{
	x = xx;
	y = yy;
	weight = ww;
	classification = cc;
}

double Point2DSample::getDimensionN(unsigned int N)
{
	if(N == 1)
		return this->getX();
	else if(N == 2)
		return this->getY();
	else
		cout<<"ERROR_getDimensionN [ dim = "<<N<<" ] - First dimenstion is 1. The last one is 2"<<endl;
}

void Point2DSample::setDimensionN(unsigned int N, double value) //Strating from 1
{
	if(N == 1)
	{
		this->setX(value);
	}
	else if(N == 2)
	{	
		this->setY(value);
	}
	else
		cout<<"ERROR_setDimensionN [ dim = "<<N<<" ] - First dimenstion is 1. The last one is 2"<<endl;
}


///////////////////////////////////////////////////////////////////////////////
//							3D-POINT
///////////////////////////////////////////////////////////////////////////////
Point3DSample::Point3DSample(double xx, double yy, double zz, double ww, char cc)
{
	x = xx;
	y = yy;
	z = zz;
	weight = ww;
	classification = cc;
}

double Point3DSample::getDimensionN(unsigned int N)
{
	if(N == 1)
		return this->getX();
	else if(N == 2)
		return this->getY();
	else if(N == 3)
		return this->getZ();
	else
		cout<<"ERROR - First dimenstion is 1. The last one is 3"<<endl;
}

void Point3DSample::setDimensionN(unsigned int N, double value) //Strating from 1
{
	if(N == 1)
		this->setX(value);
	else if(N == 2)
		this->setY(value);
	else if(N == 3)
		this->setZ(value);
	else
		cout<<"ERROR - First dimenstion is 1. The last one is 3"<<endl;
}

///////////////////////////////////////////////////////////////////////////////
//							4D-POINT
/////////////////////////////////////////////////////////////////////////////////////////////////////////
Point4DSample::Point4DSample(double xx, double yy, double zz, double jj, double ww, char cc)
{
	x = xx;
	y = yy;
	z = zz;
	j = jj;
	weight = ww;
	classification = cc;
}

double Point4DSample::getDimensionN(unsigned int N)
{
	if(N == 1)
		return this->getX();
	else if(N == 2)
		return this->getY();
	else if(N == 3)
		return this->getZ();
	else if(N == 4)
		return this->getJ();
	else
		cout<<"ERROR - First dimenstion is 1. The last one is 4"<<endl;
}

void Point4DSample::setDimensionN(unsigned int N, double value) //Strating from 1
{
	if(N == 1)
		this->setX(value);
	else if(N == 2)
		this->setY(value);
	else if(N == 3)
		this->setZ(value);
	else if(N == 4)
		this->setJ(value);
	else
		cout<<"ERROR - First dimenstion is 1. The last one is 4"<<endl;
}


///////////////////////////////////////////////////////////////////////////////
//							5D-POINT
/////////////////////////////////////////////////////////////////////////////////////////////////////////
Point5DSample::Point5DSample(double xx, double yy, double zz, double jj, double tt, double ww, char cc)
{
	x = xx;
	y = yy;
	z = zz;
	j = jj;
	t = tt;
	weight = ww;
	classification = cc;
}

double Point5DSample::getDimensionN(unsigned int N)
{
	if(N == 1)
		return this->getX();
	else if(N == 2)
		return this->getY();
	else if(N == 3)
		return this->getZ();
	else if(N == 4)
		return this->getJ();
	else if(N == 5)
		return this->getT();
	else
		cout<<"ERROR - First dimenstion is 1. The last one is 5"<<endl;
}

void Point5DSample::setDimensionN(unsigned int N, double value) //Strating from 1
{
	if(N == 1)
		this->setX(value);
	if(N == 2)
		this->setY(value);
	if(N == 3)
		this->setZ(value);
	if(N == 4)
		this->setJ(value);
	if(N == 5)
		this->setT(value);
	else
		cout<<"ERROR - First dimenstion is 1. The last one is 5"<<endl;
}


///////////////////////////////////////////////////////////////////////////////
//							ARRAY1D-2D POINT DATASET
///////////////////////////////////////////////////////////////////////////////
//Insert element into vector
Array1D2DPointDataset::Array1D2DPointDataset(double ww, char cc)
{
	weight = ww;
	classification = cc;
}

void Array1D2DPointDataset::insert(Point2DSample CS)
{
	//Insert element at the end of vector samples
    samples.push_back(CS);
}

void Array1D2DPointDataset::clear()
{
  samples.clear();
}

void Array1D2DPointDataset::eraseByClass(char c)
{
	vector<Point2DSample>::iterator SamplesIterator;
	
	for(SamplesIterator = this->begin(); SamplesIterator != this->end(); SamplesIterator++)
	{
		if (SamplesIterator->getClassification() == c)
		{
			samples.erase(SamplesIterator);
			SamplesIterator-- ;
		}
	}
}

unsigned int Array1D2DPointDataset::size()
{
  return samples.size();
}

vector<Point2DSample> Array1D2DPointDataset::interpolate(unsigned int nos, int interpolationType)
{
	vector<Point2DSample> output;
	Point2DSample temp = Point2DSample();
	
	//vector<Point2DSample>::iterator outputIt;
	
	if(interpolationType == HERMITE_INTERPOLATION)
	{
		//Interpolate value for each dimensions
		for(int dim = 1; dim <= 2; dim ++)
		{
			//Initialization
			double t = 0.0;
			double m0 = 0.0;
			double m1 = 0.0;
			double x0 = 0.0;
			double x1 = 0.0;
			
			unsigned int inputIt = 0;
			unsigned int outputIt = 0;
			
			//interpolation ratio
			double ratio= (double)this->size()/(double)nos;
			
			//normalization 
			for (outputIt = 0; outputIt < nos; outputIt++)				
			{
				
				if (outputIt == 0)
				{
					x1 = 0.0;
					if (dim == 1)
					{
						temp.setDimensionN(dim ,(this->getSample(inputIt)).getDimensionN(dim));
						output.push_back(temp);
					}
					else
					{
						(output[outputIt]).setDimensionN(dim, (this->getSample(inputIt)).getDimensionN(dim));
					}
				}
				else
				{
					//set coord of missing point
					x1 = x0 + ratio;
					
					//set index of realSamples vector
					inputIt = (unsigned int)x1;
					
					//using real point when possible
					if((unsigned int)x1 != (unsigned int)x0)
					{
						if(dim == 1)
						{
							temp.setDimensionN(dim ,(this->getSample(inputIt)).getDimensionN(dim));
							output.push_back(temp);
						}
						else
							(output[outputIt]).setDimensionN(dim, (this->getSample(inputIt)).getDimensionN(dim));
					}
					//else...interpolation
					else
					{
						t = x1 - (int)x1;
						
						//Hermite polynomials
						hermitePolynomials(t);
					
						//If current point it's the point next to last one, use last real point
						if( inputIt == this->size()-1) //VERIFICARE
						{
							m0 = angularCoefficient((this->getSample(inputIt-1)).getDimensionN(dim),
													(this->getSample(inputIt)).getDimensionN(dim),
													(this->getSample(inputIt+1)).getDimensionN(dim));
													
							m1 = angularCoefficient((this->getSample(inputIt)).getDimensionN(dim),
													(this->getSample(inputIt+1)).getDimensionN(dim),
													(this->getSample(inputIt+1)).getDimensionN(dim));
						}
						//if current point is the last one, stop interpolation
						else if(outputIt == nos)
						{
							if(dim == 1)
							{
								temp.setDimensionN(dim ,(this->getSample(inputIt)).getDimensionN(dim));
								output.push_back(temp);
							}				
							else
								(output[outputIt]).setDimensionN(dim, (this->getSample(inputIt)).getDimensionN(dim));

							break;
						}
						//otherwise
						else
						{
							m0 = angularCoefficient((this->getSample(inputIt-1)).getDimensionN(dim),
													(this->getSample(inputIt)).getDimensionN(dim),
													(this->getSample(inputIt+1)).getDimensionN(dim));
													
							m1 = angularCoefficient((this->getSample(inputIt)).getDimensionN(dim),
													(this->getSample(inputIt+1)).getDimensionN(dim),
													(this->getSample(inputIt+2)).getDimensionN(dim));
						}
						
						//Save interpolated point
						if (dim == 1)
						{
							temp.setDimensionN(dim, (float)(h00 * (this->getSample(inputIt)).getDimensionN(dim) + 
															h10 * m0 +
															h01 * (this->getSample(inputIt+1)).getDimensionN(dim) +
															h11 * m1));		
							output.push_back(temp);
							
							if((temp.getDimensionN(dim) < 0.001) && (temp.getDimensionN(dim) > -0.001))
								(output[outputIt]).setDimensionN(dim, 0.0);
							
							
						}
						else
						{
							(output[outputIt]).setDimensionN(dim, (float)(h00 * (this->getSample(inputIt)).getDimensionN(dim) + 
																						h10 * m0 +
																						h01 * (this->getSample(inputIt+1)).getDimensionN(dim) +
																						h11 * m1));	
							
							if(((output[outputIt]).getDimensionN(dim) < 0.001) && ((output[outputIt]).getDimensionN(dim) > -0.001))
								(output[outputIt]).setDimensionN(dim, 0.0);
						}
					}
				}
				x0 = x1;
			}			
		}
	}
	
	return output;
}

double Array1D2DPointDataset::angularCoefficient(double p0, double p1, double p2)
{
	return (((p2-p1)/2)+((p1-p0)/2));	
}

void Array1D2DPointDataset::hermitePolynomials(double t)
{
	h00 = 2*pow(t,3) - 3*pow(t,2) + 1;
	h10 = pow(t,3) - 2*pow(t,2) + t;
	h01 = -2*pow(t,3) + 3*pow(t,2);
	h11 = pow(t,3) - pow(t,2);
}

void Array1D2DPointDataset::save(string filename, int format, bool size, bool classification)
{
	ofstream File;
	File.open(filename.c_str(),ios::out);
	vector<Point2DSample>::iterator SamplesIterator;
	
	if(size) { 	File << this->size()<<' '<<endl; }
	
	if(format == SAVE_AS_INT)
	for(SamplesIterator = this->begin(); SamplesIterator != this->end(); SamplesIterator++)
	{
	  File << (int)SamplesIterator->getX() << ' ';
	  File << (int)SamplesIterator->getY();
	   if (classification)
	  	File << ' ' << (char)SamplesIterator->getClassification() <<endl;
	  else
	  	File <<endl;
	}
	else if(format == SAVE_AS_FLOAT)
	for(SamplesIterator = this->begin(); SamplesIterator != this->end(); SamplesIterator++)
	{
	  File << (float)SamplesIterator->getX() << ' ';
	  File << (float)SamplesIterator->getY();
	   if (classification)
	  	File << ' ' << (char)SamplesIterator->getClassification() <<endl;
	  else
	  	File <<endl;
	}
	else
	for(SamplesIterator = this->begin(); SamplesIterator != this->end(); SamplesIterator++)
	{
	  File << (double)SamplesIterator->getX() << ' ';
	  File << (double)SamplesIterator->getY();
	   if (classification)
	  	File << ' ' << (char)SamplesIterator->getClassification() <<endl;
	  else
	  	File <<endl;
	  
	}

  File.close();   	 
}

void Array1D2DPointDataset::load(string filename, int format, bool size)
{
	
}


///////////////////////////////////////////////////////////////////////////////
//							ARRAY1D-3DPOINT-DATASET
///////////////////////////////////////////////////////////////////////////////
//Insert element into vector
Array1D3DPointDataset::Array1D3DPointDataset(double ww, char cc)
{
	weight = ww;
	classification = cc;
}

void Array1D3DPointDataset::insert(Point3DSample CS)
{
	//Insert element at the end of vector samples
  samples.push_back(CS);
}

void Array1D3DPointDataset::eraseByClass(char c)
{
	vector<Point3DSample>::iterator SamplesIterator;
	
	for(SamplesIterator = this->begin(); SamplesIterator != this->end(); SamplesIterator++)
	{
		if (SamplesIterator->getClassification() == c)
		{
			samples.erase(SamplesIterator);
			SamplesIterator-- ;
		}
	}
}

void Array1D3DPointDataset::clear()
{
  samples.clear();
}

unsigned int Array1D3DPointDataset::size()
{
  return samples.size();
}

void Array1D3DPointDataset::save(string filename, int format, bool size, bool classification)
{
	ofstream File;
	File.open(filename.c_str(),ios::out);
	vector<Point3DSample>::iterator SamplesIterator;
	
	if(size) { 	File << this->size()<<' '<<endl; }
	
	if(format == SAVE_AS_INT)
	for(SamplesIterator = this->begin(); SamplesIterator != this->end(); SamplesIterator++)
	{
	  File << (int)SamplesIterator->getX() << ' ';
	  File << (int)SamplesIterator->getY() << ' ';
	  File << (int)SamplesIterator->getZ();
	  if (classification)
	  	File << ' ' << (char)SamplesIterator->getClassification() <<endl;
	  else
	  	File <<endl;
	}
	else if(format == SAVE_AS_FLOAT)
	for(SamplesIterator = this->begin(); SamplesIterator != this->end(); SamplesIterator++)
	{
	  File << (float)SamplesIterator->getX() << ' ';
	  File << (float)SamplesIterator->getY() << ' ';
	  File << (float)SamplesIterator->getZ();
	  if (classification)
	  	File << ' ' << (char)SamplesIterator->getClassification() <<endl;
	  else
	  	File <<endl;
	}
	else
	for(SamplesIterator = this->begin(); SamplesIterator != this->end(); SamplesIterator++)
	{
	  File << (double)SamplesIterator->getX() << ' ';
	  File << (double)SamplesIterator->getY() << ' ';
	  File << (double)SamplesIterator->getZ();
	  if (classification)
	  	File << ' ' << (char)SamplesIterator->getClassification() <<endl;
	  else
	  	File <<endl;
	}

  File.close();   	 
}

void Array1D3DPointDataset::load(string filename, int format, bool size, bool classification)
{
	int modelSize = 1;
	char classLabel = 'U';
	double X, Y, Z;
	Point3DSample actualSample;
	 
	//File
	ifstream File;
	File.open(filename.c_str(),ios::in);

	//Read model dimension
	if(size)
		File >> modelSize;
	
	for ( int i =0; i< (int)modelSize; i++)
	{
		//Read point
		File >> X;
		File >> Y;
		File >> Z;
		
		cout<<"READED DATA"<<X<<" "<<Y<<" "<<Z<<" "<<endl;
		
		//Read class
		if(classification)
			File >> classLabel;
		
		if(format == LOAD_AS_DOUBLE)
			actualSample = Point3DSample((double)X,(double)Y,(double)Z, 1.0, classLabel);
		else if(format == LOAD_AS_FLOAT)
			actualSample = Point3DSample((float)X,(float)Y,(float)Z, 1.0, classLabel);
		else
			actualSample = Point3DSample((int)X,(int)Y,(int)Z, 1.0, classLabel);
		this->insert(actualSample);	
		
	}
	
	File.close();   	 
}


///////////////////////////////////////////////////////////////////////////////
//							ARRAY1D-4DPOINT-DATASET
///////////////////////////////////////////////////////////////////////////////
//Insert element into vector
Array1D4DPointDataset::Array1D4DPointDataset(double ww, char cc)
{
	weight = ww;
	classification = cc;
}

void Array1D4DPointDataset::insert(Point4DSample CS)
{
	//Insert element at the end of vector samples
  samples.push_back(CS);
}

void Array1D4DPointDataset::eraseByClass(char c)
{
	vector<Point4DSample>::iterator SamplesIterator;
	
	for(SamplesIterator = this->begin(); SamplesIterator != this->end(); SamplesIterator++)
	{
		if (SamplesIterator->getClassification() == c)
		{
			samples.erase(SamplesIterator);
			SamplesIterator-- ;
		}
	}
}

void Array1D4DPointDataset::clear()
{
  samples.clear();
}

unsigned int Array1D4DPointDataset::size()
{
  return samples.size();
}

void Array1D4DPointDataset::save(string filename, int format, bool size, bool classification)
{
	ofstream File;
	File.open(filename.c_str(),ios::out);
	vector<Point4DSample>::iterator SamplesIterator;
	
	if(size) { 	File << this->size()<<' '<<endl; }
	
	if(format == SAVE_AS_INT)
	for(SamplesIterator = this->begin(); SamplesIterator != this->end(); SamplesIterator++)
	{
	  File << (int)SamplesIterator->getX() << ' ';
	  File << (int)SamplesIterator->getY() << ' ';
	  File << (int)SamplesIterator->getZ() << ' ';
	  File << (int)SamplesIterator->getJ();
	  if (classification)
	  	File << ' ' << (char)SamplesIterator->getClassification() <<endl;
	  else
	  	File <<endl;
	}
	else if(format == SAVE_AS_FLOAT)
	for(SamplesIterator = this->begin(); SamplesIterator != this->end(); SamplesIterator++)
	{
	  File << (float)SamplesIterator->getX() << ' ';
	 	File << (float)SamplesIterator->getY() << ' ';
	  File << (float)SamplesIterator->getZ() << ' ';
	  File << (float)SamplesIterator->getJ();
	  if (classification)
	  	File << ' ' << (char)SamplesIterator->getClassification() <<endl;
	  else
	  	File <<endl;
	}
	else
	for(SamplesIterator = this->begin(); SamplesIterator != this->end(); SamplesIterator++)
	{
	  File << (double)SamplesIterator->getX() << ' ';
	  File << (double)SamplesIterator->getY() << ' ';
	  File << (double)SamplesIterator->getZ() << ' ';
	  File << (double)SamplesIterator->getJ();
	  if (classification)
	  	File << ' ' << (char)SamplesIterator->getClassification() <<endl;
	  else
	  	File <<endl;
	}

  File.close();   	 
}

void Array1D4DPointDataset::load(string filename, int format, bool size, bool classification)
{
	int modelSize = 1;
	char classLabel = 'U';
	double X, Y, Z, J;
	Point4DSample actualSample;
	 
	//File
	ifstream File;
	File.open(filename.c_str(),ios::in);

	//Read model dimension
	if(size)
		File >> modelSize;
	
	for ( int i =0; i< (int)modelSize; i++)
	{
		//Read point
		File >> X;
		File >> Y;
		File >> Z;
		File >> J;
		
		cout<<"READED DATA"<<X<<" "<<Y<<" "<<Z<<" "<<J<<" "<<endl;
		
		//Read class
		if(classification)
			File >> classLabel;
		
		if(format == LOAD_AS_DOUBLE)
			actualSample = Point4DSample((double)X,(double)Y,(double)Z, (double)J, 1.0, classLabel);
		else if(format == LOAD_AS_FLOAT)
			actualSample = Point4DSample((float)X,(float)Y,(float)Z, (float)J, 1.0, classLabel);
		else
			actualSample = Point4DSample((int)X,(int)Y,(int)Z, (int)J, 1.0, classLabel);
		this->insert(actualSample);	
		
	}
	
	File.close();   	 
}


///////////////////////////////////////////////////////////////////////////////
//							ARRAY1D-5DPOINT-DATASET
///////////////////////////////////////////////////////////////////////////////
//Insert element into vector
Array1D5DPointDataset::Array1D5DPointDataset(double ww, char cc)
{
	weight = ww;
	classification = cc;
}

void Array1D5DPointDataset::insert(Point5DSample CS)
{
	//Insert element at the end of vector samples
  samples.push_back(CS);
}

void Array1D5DPointDataset::eraseByClass(char c)
{
	vector<Point5DSample>::iterator SamplesIterator;
	
	for(SamplesIterator = this->begin(); SamplesIterator != this->end(); SamplesIterator++)
	{
		if (SamplesIterator->getClassification() == c)
		{
			samples.erase(SamplesIterator);
			SamplesIterator-- ;
		}
	}
}

void Array1D5DPointDataset::clear()
{
  samples.clear();
}

unsigned int Array1D5DPointDataset::size()
{
  return samples.size();
}

void Array1D5DPointDataset::save(string filename, int format, bool size, bool classification)
{
	ofstream File;
	File.open(filename.c_str(),ios::out);
	vector<Point5DSample>::iterator SamplesIterator;
	
	if(size) { 	File << this->size()<<' '<<endl; }
	
	if(format == SAVE_AS_INT)
	for(SamplesIterator = this->begin(); SamplesIterator != this->end(); SamplesIterator++)
	{
	  File << (int)SamplesIterator->getX() << ' ';
	  File << (int)SamplesIterator->getY() << ' ';
	  File << (int)SamplesIterator->getZ() << ' ';
	  File << (int)SamplesIterator->getJ() << ' ';
	  File << (int)SamplesIterator->getT();
	  if (classification)
	  	File << ' ' << (char)SamplesIterator->getClassification() <<endl;
	  else
	  	File <<endl;
	}
	else if(format == SAVE_AS_FLOAT)
	for(SamplesIterator = this->begin(); SamplesIterator != this->end(); SamplesIterator++)
	{
	  File << (float)SamplesIterator->getX() << ' ';
	 	File << (float)SamplesIterator->getY() << ' ';
	  File << (float)SamplesIterator->getZ() << ' ';
	  File << (float)SamplesIterator->getJ() << ' ';
	  File << (float)SamplesIterator->getT();
	  if (classification)
	  	File << ' ' << (char)SamplesIterator->getClassification() <<endl;
	  else
	  	File <<endl;
	}
	else
	for(SamplesIterator = this->begin(); SamplesIterator != this->end(); SamplesIterator++)
	{
	  File << (double)SamplesIterator->getX() << ' ';
	  File << (double)SamplesIterator->getY() << ' ';
	  File << (double)SamplesIterator->getZ() << ' ';
	  File << (double)SamplesIterator->getJ() << ' ';
	  File << (double)SamplesIterator->getT();
	  if (classification)
	  	File << ' ' << (char)SamplesIterator->getClassification() <<endl;
	  else
	  	File <<endl;
	}

  File.close();   	 
}

void Array1D5DPointDataset::load(string filename, int format, bool size, bool classification)
{
	int modelSize = 1;
	char classLabel = 'U';
	double X, Y, Z, J, T;
	Point5DSample actualSample;
	 
	//File
	ifstream File;
	File.open(filename.c_str(),ios::in);

	//Read model dimension
	if(size)
		File >> modelSize;
	
	for ( int i =0; i< (int)modelSize; i++)
	{
		//Read point
		File >> X;
		File >> Y;
		File >> Z;
		File >> J;
		File >> T;
		
		cout<<"READED DATA"<<X<<" "<<Y<<" "<<Z<<" "<<J<<" "<<T<<endl;
		
		//Read class
		if(classification)
			File >> classLabel;
		
		if(format == LOAD_AS_DOUBLE)
			actualSample = Point5DSample((double)X,(double)Y,(double)Z, (double)J, (double)T, 1.0, classLabel);
		else if(format == LOAD_AS_FLOAT)
			actualSample = Point5DSample((float)X,(float)Y,(float)Z, (float)J, (float)T, 1.0, classLabel);
		else
			actualSample = Point5DSample((int)X,(int)Y,(int)Z, (int)J, (int)T, 1.0, classLabel);
		this->insert(actualSample);	
		
	}
	
	File.close();   	 
}


///////////////////////////////////////////////////////////////////////////////
//							ARRAY2D-2D POINT DATASET
///////////////////////////////////////////////////////////////////////////////
//Insert element into vector
Array2D2DPointDataset::Array2D2DPointDataset(double ww, char cc)
{
	weight = ww;
	classification = cc;
}

void Array2D2DPointDataset::insert(Array1D2DPointDataset AD)
{
	//Insert element at the end of vector samples
  arraySamples.push_back(AD);
}

void Array2D2DPointDataset::clear()
{
  arraySamples.clear();
}

unsigned int Array2D2DPointDataset::size()
{
  return arraySamples.size();
}


void Array2D2DPointDataset::save(string filename, int format, bool size, bool classification)
{
	
}

void Array2D2DPointDataset::load(string filename, int format, bool size)
{
	
}


///////////////////////////////////////////////////////////////////////////////
//							ARRAY2D-4DPOINT-DATASET
///////////////////////////////////////////////////////////////////////////////
//Insert element into vector
Array2D4DPointDataset::Array2D4DPointDataset(double ww, char cc)
{
	weight = ww;
	classification = cc;
}

void Array2D4DPointDataset::insert(Array1D4DPointDataset AD)
{
	//Insert element at the end of vector samples
	arraySamples.push_back(AD);
}

void Array2D4DPointDataset::clear()
{
  arraySamples.clear();
}

unsigned int Array2D4DPointDataset::size()
{
  return arraySamples.size();
}


void Array2D4DPointDataset::save(string filename, int format, bool size, bool classification)
{
	
}

void Array2D4DPointDataset::load(string filename, int format, bool size)
{
	
}
