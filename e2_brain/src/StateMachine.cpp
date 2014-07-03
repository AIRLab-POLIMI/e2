/*
Author: Deborah Zamponi
Mail: deborahzamponi@gmail.com
Data: 23/01/2012
Library: 
*/

#include "StateMachine.h"
#include <iostream>


StateMachine::StateMachine()
{}

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


Node* StateMachine::getNode(string name)
{
	for(int i = 0; i < nodes.size(); i++)
	{
		if((nodes[i].getName()).compare(name) == 0)
		 return &nodes[i];
	}
	
	return NULL;
}


Node* StateMachine::getCurrent()
{
	return getNode(current);
}

string StateMachine::removeSpacesAndTab(string source)
{	
	while( source.at(0) == ' ' || source.at(0) == '\t' )
	{
		source.erase(0,1);
	}
	
	return source;
}

int StateMachine::loadFromFile(string file)
{
	string fileLine;
	size_t found;
	unsigned int nodeElemCounter = 0;
	bool nodeReady = false;
	int err;

	//Open file
	ifstream File;
	File.open(file.c_str(), ios::in);

	while(!File.eof())
	{
		//Count if on current line there is all the node elements (4)
		getline(File, fileLine);
		
		//Check if is comment line (looking for for some "#")
		found = fileLine.find_first_of("#");
		if( found > 0)
		{
			if( fileLine.compare("---") != 0 && nodeReady == false)
			{
				//Get first parameter
				found = fileLine.find_first_of(";");
				if(found <= 0) cout<<endl<<"Error parsing state machines file"<<endl;
				else
				{
					string nID = removeSpacesAndTab(fileLine.substr(0, found));
					fileLine.erase(0, found+1);
					
					//Get second parameter
					found = fileLine.find_first_of(";");
					if(found <= 0) cout<<endl<<"Error parsing state machines file"<<endl;
					else
					{
						string nName = removeSpacesAndTab(fileLine.substr(0, found));						
						fileLine.erase(0, found+1);
						
						//Get third parameter
						found = fileLine.find_first_of(";");
						if(found <= 0) cout<<endl<<"Error parsing state machines file"<<endl;
						else
						{
							string nOutput = removeSpacesAndTab(fileLine.substr(0, found));
							fileLine.erase(0, found+1);
						
							//Get fourth parameter
							found = fileLine.find_first_of(";");
							if(found <= 0) cout<<endl<<"Error parsing state machines file"<<endl;
							else
							{
								string nParameter = removeSpacesAndTab(fileLine.substr(0, found));
								nodeElemCounter ++;
								
								//Add current node into State Machine
								stringstream ss(nID);
								int temp;
								ss >> temp;
								Node n = Node(temp, nName, nOutput, nParameter);
								addNode(n);
								if(nodeElemCounter == 1) {this->current = temp;}
							}
						}
					}
				}
			}
			else if( fileLine.compare("---") == 0 && nodeReady == false)  //"---" founded
			{  
				nodeReady = true;
				cout<<"----"<<endl;
			}
			
			else if( fileLine.compare("---") != 0 && nodeReady == true)  //Transiction part
			{
				//Get first parameter
				found = fileLine.find_first_of(";");
				if(found <= 0) cout<<endl<<"Error parsing state machines file"<<endl;
				else
				{
					string startName = removeSpacesAndTab(fileLine.substr(0, found));						
					fileLine.erase(0, found+1);
					
					//Get second parameter
					found = fileLine.find_first_of(";");
					if(found <= 0) cout<<endl<<"Error parsing state machines file"<<endl;
					else
					{
						string endName = removeSpacesAndTab(fileLine.substr(0, found));						
						fileLine.erase(0, found+1);
						
						//Get third parameter
						found = fileLine.find_first_of(";");
						if(found <= 0) cout<<endl<<"Error parsing state machines file"<<endl;
						else
						{
							string transiction = removeSpacesAndTab(fileLine.substr(0, found));						
							fileLine.erase(0, found+1);
							
							//Add transiction into State Machine
							stringstream ss(transiction);
							int temp;
							ss >> temp;
							err = setTransictionBt(startName, endName, temp);
							if(err < 0) cout<<endl<<"Error parsing state machines file"<<endl;
						}
					}
				}
			}
			else if( fileLine.compare("---") == 0 && nodeReady == true) //EndFile
			{
				break;
			}
		}
		else{} //Comment line founded
	}	
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
	if (temp != NULL)
		current = temp->getID();
	return temp;
}


int StateMachine::setTransictionBt(string src, string dest, int t)
{
	if(getNode(src) == NULL || getNode(dest) == NULL)
		return -1;
	else
	{
		getNode(src)->setNextNode(t, getNode(dest));
		return 1;
	}
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



