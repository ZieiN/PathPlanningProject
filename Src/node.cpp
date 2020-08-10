#include "node.h"
#include <iostream>

Node::Node(){
	i=j=F=g=H=0;
	parent=NULL;
}
Node::Node(int i, int j, double F, double g, double H, Node *parent){
	this->i=i;
	this->j=j;
	this->F=F;
	this->g=g;
	this->H=H;
	this->parent=parent;
}
