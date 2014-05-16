/*
Author: Cristian Mandelli e Deborah Zamponi
Mail: cristianmandelli@gmail.com
* 	  deborahzamponi@gmail.com
Data: 4/1/2012
Library: 
*/

#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <ann/ANN.h>
#include "PointNDDataset.h"

using namespace std;

typedef enum {KNN_CLASSIC, KNN_WITH_RADIUS} knnType;
typedef enum {WITHOUT_LOG, WITH_LOG} knnLogInfo;


struct classScore
{
	unsigned char c;
	unsigned int score;
};

/*
 * KNNClassifier.
 * 
 */
class KNNClassifier
{
	private:
	//Structure that stores score of a classification
	vector<classScore> scores;
	
	//This class stores all the informations used to build the KNNTree
	ANNpointArray model;
	//This class stores the point how you need the K nearest neighbors 
	//points
	ANNpoint queryPoint;
	//This class stores the indexes of point that correspond to the 
	//nearest neighbors founded into model
	ANNidxArray idxs;
	//This class stores the distances between current queryPoint and 
	//the corrensponding k-i point founded into model
	ANNdistArray dists;
	//Search tree
	ANNkd_tree *KccTree;
	
	//Number of point into model (first numer of file)
	unsigned int modelPts;
	unsigned int k;
	
	//This array stores class point into model (in order with it)
	unsigned char* classPoints;
	
	//file descriptor
	ifstream fdData;
	ifstream fdPoint;
	
	//reset class score
	void resetScore();
	
	public:
	//Costructor
	KNNClassifier();
	
	//This class creates a tree with model points and implements a
	//faster way to find the nerarest neightbors. filename is file in 
	//which model point are stored, dist is 
	void fastBuild(string filename, unsigned int kk, int dist, int dim);
	
	//The following methods return query point class (you must use the correct point dimensions)
	unsigned char getClassification(vector<Point2DSample> data, vector<int> &returnScores, vector<char> &returnClass, int type=KNN_CLASSIC, float sqRadius=-1, int log = WITHOUT_LOG); //dim = 2
	//unsigned char getClassificationScores(vector<Point2DSample> data, vector<unsigned int> returnScore, vector<unsigned char> returnClass, int type=KNN_CLASSIC, float sqRadius=-1, int log = WITHOUT_LOG); //dim = 2

	//unsigned char getClassification(Array1D3DPointDataset data, int type=KNN_CLASSIC, float sqRadius=-1, int log = WITHOUT_LOG); //dim = 3
	//unsigned char getClassification(Array1D4DPointDataset data, int type=KNN_CLASSIC, float sqRadius=-1, int log = WITHOUT_LOG); //dim = 4
	
};





















