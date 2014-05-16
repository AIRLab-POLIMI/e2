/*
Author: Cristian Mandelli e Deborah Zamponi
Mail: cristianmandelli@gmail.com
* 	  deborahzamponi@gmail.com
Data: 4/1/2012
Library: 
*/

#include "Classifiers.h"

///////////////////////////////////////////////////////////////////////////////
//							KNN-CLASSIFIER
///////////////////////////////////////////////////////////////////////////////
KNNClassifier::KNNClassifier()
{
}


void KNNClassifier::fastBuild(string filename, unsigned int kk, int dist, int dim)
{
	//Save nearest number
	k = kk;
	
	//Load information from file
	fdData.open(filename.c_str(), ios::in);
	//First file value is the number of model points
	fdData >> modelPts;
	
	cout<<endl<<"Crating model...";
	//Data structures allocation
	classPoints = (unsigned char*)malloc(modelPts*sizeof(unsigned char));
	model = annAllocPts(modelPts, dim);
	queryPoint = annAllocPt(dim);
	idxs = new ANNidx[k];
	dists = new ANNdist[k];
	cout<<"...[OK]"<<endl;
	
	//Read model data from file
	cout<<endl<<"Reading model from file..."<<endl;
	for (int i = 0; i < modelPts; i++)
	{
		//Load point dimensions
		for (int j = 0; j < dim; j++)
			fdData >> model[i][j];
		
		//Load point class
		fdData >> classPoints[i];
		
		//Save classes that are into the model
		int temp = -1;
		if(scores.size() == 0)
		{
			classScore cs;
			cs.c = classPoints[i];
			cs.score = 0;
			scores.push_back(cs);
			cout<<"Classe Letta:"<<cs.c<<endl;

		}
		for(int k = 0; k < scores.size(); k++) //Search if class are already into vector
		{
			if(scores[k].c == classPoints[i]){ temp = -1; break; }
			else {temp = 0;}
		}
		if(temp == 0)
		{
			classScore cs;
			cs.c = classPoints[i];
			cs.score = 0;
			scores.push_back(cs);
			cout<<"Classe Letta:"<<cs.c<<endl;
		}
	}
	cout<<"...[OK]"<<endl;
	
	fdData.close();
	
	//Building tree
	cout<<endl<<"Building tree...";
	KccTree = new ANNkd_tree(model, modelPts, dim);
	cout<<"...[OK]"<<endl;
}

unsigned char KNNClassifier::getClassification(vector<Point2DSample> data, vector<int> &returnScores, vector<char> &returnClass, int type, float sqRadius, int log)
{
	//Variables
	unsigned int numQueryPoints = data.size();
	//cout<<"data.size = "<<numQueryPoints;
	unsigned int dim = 2;
	
	double eps;
	
	if(type == KNN_CLASSIC)
	{
		return ('U');
	}
	else if (type == KNN_WITH_RADIUS)
	{
		if(log == WITH_LOG)
			cout<<endl<<"KNN_WITH_RADIUS classification...";
		
		//For each point into query points
		for(int i = 0; i < numQueryPoints; i++)
		{
			//Read one point
			for(int j = 0; j < dim; j++)
			{
				queryPoint[j] = (data[i]).getDimensionN(j+1);
			}
		
			//Classify current point
			ANNdist radius = sqRadius;
			KccTree->annkFRSearch(queryPoint, radius, k, idxs, dists, eps);
			
			//Point class equals to most frequent class into k nearest neighbors point
			//returned by previous function
			for(int w = 0; w < k; w++)
			{
				if(idxs[w] == ANN_NULL_IDX){}
				else
				{
					//count score
					for(int z = 0; z < scores.size(); z++)
					{
						
						if(scores[z].c == classPoints[idxs[w]])
							scores[z].score ++;
					}
				}
			}//ending scores computation	
		}//ending query points classification
		
		//Founding most frequent class that is the class of query points
		int temp = scores[0].score;
		int tempIdx = 0;
		if(log == WITH_LOG)
		{
			cout<<endl<<"SCORES"<<endl;
			cout<<"=================================="<<endl;
		}
		for(int i=0; i < scores.size(); i++)
		{	
			//save scores of each class
			returnScores.push_back((int)scores[i].score);
			returnClass.push_back((char)scores[i].c);
			
			//detect best class
			if(log == WITH_LOG)
				cout<<"Score of class "<<scores[i].c<<" = "<<scores[i].score<<endl;
			
			if(scores[i].score > temp)
				tempIdx = i;	
		}
		if(log == WITH_LOG)
		{
			cout<<"QUERY POINT CLASS: "<<scores[tempIdx].c<<endl;
		}
		
		unsigned char result = scores[tempIdx].c;
		
		resetScore();		
		return (result);
	}
	
	else
	{
		cout<<"Error:: no classification method named "<<type<<endl;
		return ('U');
	}
	

}



//reset class score
void KNNClassifier::resetScore()
{
	for(int i = 0; i < scores.size(); i++)
	{
		scores[i].score = 0;
	}
}




