#include "node.h"
#include <iostream>

Node::Node(){
	i=j=F=g=H=parent=0;
}
Node::Node(int i, int j, double F, double g, double H, int parent){
	this->i=i;
	this->j=j;
	this->F=F;
	this->g=g;
	this->H=H;
	this->parent=parent;
}
