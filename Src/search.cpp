#include "search.h"
#include "gl_const.h"
#include "map.h"
#include "node.h"
#include "mission.h"
#include <bits/stdc++.h>
using namespace std;
Search::Search()
{
//set defaults here
}

Search::~Search() {
	delete CLOSED;
	for(int i=0; i<h+2; ++i){
		delete[] vis[i];
	}
	delete[] vis;
}

double Search::EstimateCost(int sti, int stj, int ndi, int ndj, const EnvironmentOptions &options){
	int dif_mn=abs(sti-ndi), dif_mx=abs(stj-ndj);
	if(dif_mn>dif_mx){
		swap(dif_mx, dif_mn);
	}
	if(options.metrictype==CN_SP_MT_DIAG){
		return hypot(dif_mn, dif_mn)+dif_mx-dif_mn;
	}
	if(options.metrictype==CN_SP_MT_EUCL){
		return hypot(dif_mn, dif_mx);
	}
	if(options.metrictype==CN_SP_MT_MANH){
		return dif_mn+dif_mx;
	}
	if(options.metrictype==CN_SP_MT_CHEB){
		return dif_mx;
	}
}
bool Search::valid_move(int i1, int j1, int i2, int j2, const Map &map, const EnvironmentOptions &options){
	if(!map.CellOnGrid(i2, j2)) return false;
	if(map.CellIsObstacle(i2,j2)) return false;
	if(abs(i1-i2)+abs(j1-j2)==1) return true;
	if(options.allowdiagonal){
		int obstacles = map.CellIsObstacle(i1, j2)+map.CellIsObstacle(i2, j1);
		if(obstacles==0) return true;
		if(!options.cutcorners) return false; // at least one obtacle
		if(obstacles==1) return true;
		if(!options.allowsqueeze) return false; // there are two obstacles
		return true;
	}
	return false;
}
SearchResult Search::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
	 	int sti=map.getStart_i();
	 	int stj=map.getStart_j();
	 	int ndi=map.getGoal_i();
	 	int ndj=map.getGoal_j();
	 	h=map.getMapHeight();
	 	w=map.getMapWidth();
    Node top=Node(sti, stj, EstimateCost(sti, stj, ndi, ndj, options), 0, EstimateCost(sti, stj, ndi, ndj, options), NULL);
    q.push(top);
    int num_moves;
    if(options.allowdiagonal){
    	num_moves=8;
    }
    else{
    	num_moves=4;
    }
    CLOSED = new Node [h*w+10];
    int cnt=-1;
    vis = new bool *[h+2];
    for(int i=0; i<h+2; ++i){
    	vis[i] = new bool [w+2];
    	for(int j=0; j<w; ++j){
    		vis[i][j]=0;
    	}
    }
    sresult.pathfound=false;
    sresult.nodescreated=0;
		sresult.numberofsteps=0;
		double start_time=1.0*clock()/CLOCKS_PER_SEC;
		while(!q.empty()){
			top=q.top();
			q.pop();
			++sresult.numberofsteps;
			if(vis[top.i][top.j])continue;
			vis[top.i][top.j]=1;
			CLOSED[++cnt]=top;
			if(top.i==ndi && top.j==ndj){
				sresult.pathfound=true;
				sresult.pathlength=top.g;
				while(1){
					hppath.push_front(top);
					lppath.push_front(top);
					if(top.parent==NULL)
						break;
					top=*top.parent;
				}
				break;
			}
			for(int i=0; i<num_moves; ++i){
				int ii=top.i+diri[i];
				int jj=top.j+dirj[i];
				if(valid_move(top.i, top.j, ii, jj, map, options)){
					sresult.nodescreated++;
					double cost=hypot(ii-top.i, jj-top.j);
					double estimated_cost=EstimateCost(ii, jj, ndi, ndj, options);
					q.push(Node(ii, jj, top.g+cost+estimated_cost, top.g+cost, estimated_cost, &CLOSED[cnt]));
				}
			}
		}
		sresult.time=1.0*clock()/CLOCKS_PER_SEC-start_time;
    sresult.hppath = &hppath; //Here is a constant pointer
    sresult.lppath = &lppath;
    return sresult;
}

/*void Search::makePrimaryPath(Node curNode)
{
    //need to implement
}*/

/*void Search::makeSecondaryPath()
{
    //need to implement
}*/