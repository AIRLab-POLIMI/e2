/*
Author: Deborah Zamponi
Mail: deborahzamponi@gmail.com
Data: 21/01/2012
Library: 
*/

#include <iostream>
#include <vector>
#include <map>

using namespace std;

/*This class rappresent a node of interaction map
*/
class Node
{
	private:
	int id;                //identifier of each node
	string name;					//name of current node
	string output;                     //value to identify the primitive to call
	string params;
	map<int, Node*> next;     //container in which save pointers to next nodes, 
	                                //every pointer is characterized by an id (short int) 
	
	int getNumNext() const { return (int)next.size(); }
  
	public:
	//Constructors
	Node(int ID, string nominative, string out, string par);
	
	//Getters
	int getID() const { return id; }
	string getName() const { return name; }
	string getOutput() const { return output; }
	string getParams() const { return params; }
	
	
	Node* getNextNode(int nodeKey);   //use map::find member function
	
	//Setters
	void setNextNode(int key, Node* nextNode);
	void setNextNodes(vector<Node*> nextNodes);
	
	//Other functions
	void displayNode();
};

