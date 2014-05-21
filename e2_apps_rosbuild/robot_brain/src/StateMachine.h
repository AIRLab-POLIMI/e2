/*
Author: Deborah Zamponi
Mail: deborahzamponi@gmail.com
Data: 23/01/2012
Library: 
*/

#include "Node.h"
#include <iostream>

using namespace std;

/*This class rappresent the interaction map
*/
class StateMachine
{
	private:
	vector<Node> nodes;
	int current;
	
	//private function
	Node* getNode(int id);
	Node* getCurrent();
	
	public:
	//Constructors
	StateMachine(Node firstNode);
	
	//Other Functions
	void addNode(Node node);
	void addNodes(vector<Node> nn);
	Node* run(int input);
	int setTransictionBt(int src, int dest, int t);
	
	void printMap();
};
