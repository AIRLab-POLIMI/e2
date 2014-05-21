/*
Author: Deborah Zamponi
Mail: deborahzamponi@gmail.com
Data: 23/01/2012
Library: 
*/

#include "StateMachine.h"
#include <iostream>



StateMachine::StateMachine(Node firstNode)
{
	//creation of initial node
	current = firstNode.getID();
	nodes.push_back(firstNode);
	
	cout<<"State Machine Created!..."<<endl;
}

Node* StateMachine::getNode(int id)
{
	for(int i = 0; i < nodes.size(); i++)
	{
		if(nodes[i].getID() == id)
		 return &nodes[i];
	}
	
	return NULL;
}


Node* StateMachine::getCurrent()
{
	return getNode(current);
}

void StateMachine::addNode(Node node)
{
	nodes.push_back(node);
}

void StateMachine::addNodes(vector<Node> nn)
{
	for(int i = 0; i < nn.size(); i++)
	{
		nodes.push_back(nn[i]);
	}
}

Node* StateMachine::run(int input)
{
	Node* temp = getCurrent()->getNextNode(input);
	current = temp->getID();
	return temp;
	
}

int StateMachine::setTransictionBt(int src, int dest, int t)
{
	if ((getNode(src) == NULL) || (getNode(dest) == NULL))
	{
		return -1;
	}
	else
	{
		getNode(src)->setNextNode(t, getNode(dest));
		return 1;
	}
}

void StateMachine::printMap()
{
	for(int i = 0; i < nodes.size(); i++)
	{
		nodes[i].displayNode();
	}
}



