#include "node.h"
#include <iostream>

bool operator < (Node a, Node b){
	if(a.F==b.F)
		return a.H > b.H;
	return a.F > b.F;
}
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
