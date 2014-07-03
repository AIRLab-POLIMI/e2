/*
Author: Deborah Zamponi
Mail: deborahzamponi@gmail.com
Data: 22/01/2012
Library: 
*/

#include "Node.h"
#include <iostream>


using namespace std;

Node::Node(int ID, string nominative, string out, string par)
{
		id = ID;
		name = nominative;
		output = out;
		params = par;
}

Node* Node::getNextNode(int nodeKey)
{
	if(next.count(nodeKey) == 1)
		return (next.find(nodeKey)->second);
	return NULL;
}

void Node::setNextNode(int key, Node* nextNode)
{
	map<int, Node*>::iterator it;
	
	it = next.begin();
	next.insert(it, pair<int, Node*>(key, nextNode));
}

void Node::setNextNodes(vector<Node*> nextNodes)
{
	map<int, Node*>::iterator it;
	int key = next.size();
	
	it = next.end();
	
	for(int i = 0; i < nextNodes.size(); i++ )
	{
		key += 1;
		next.insert(it, pair<int, Node*>((int)key, nextNodes.at(i)));
	}
}

void Node::displayNode()
{
	map<int, Node*>::iterator it;
	
	cout<<endl<< "-----------------------------------------------------"<<endl;
	cout<<"Features of current node:"<<endl;
	cout<<"		ID: "<<id<<endl;
	cout<<"		Name: "<<name<<endl;
	cout<<"		Output: "<<output<<endl;
	cout<<"		Params: "<<params<<endl;
	for (it = next.begin(); it!= next.end(); it++)
	{
		cout<<"NODE: "<<this->getID()<<"------ ["<<it->first<<"] ----->"<< (it->second)->getID()<<endl;
	}
	cout<<"-----------------------------------------------------"<<endl;
	
	
	
}
