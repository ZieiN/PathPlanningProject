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
	for(int i=0; i<h+2; ++i){
		delete[] vis[i];
	}
	delete[] vis;
}
int BT, ST;
bool operator < (Node a, Node b){
	if(ST==CN_SP_ST_DIJK)
		return a.g > b.g;
	if(a.F==b.F){
		if(BT == CN_SP_BT_GMIN)
			return a.g > b.g;
		else
			return a.g < b.g;
	}
	return a.F > b.F;
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

void Search::go_BFS(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
	int sti=map.getStart_i();
 	int stj=map.getStart_j();
 	int ndi=map.getGoal_i();
 	int ndj=map.getGoal_j();
 	h=map.getMapHeight();
 	w=map.getMapWidth();
  Node front=Node(sti, stj, EstimateCost(sti, stj, ndi, ndj, options)*options.hweight, 0, EstimateCost(sti, stj, ndi, ndj, options), NULL);
  q1.push(front);
  int num_moves;
  if(options.allowdiagonal){
  	num_moves=8;
  }
  else{
  	num_moves=4;
  }
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
	while(!q1.empty()){
		front=q1.front();
		q1.pop();
		++sresult.numberofsteps;
		if(vis[front.i][front.j])continue;
		vis[front.i][front.j]=1;
		CLOSED.push_back(front);
		++cnt;
		if(front.i==ndi && front.j==ndj){
			sresult.pathfound=true;
			sresult.pathlength=front.g;
			while(1){
				hppath.push_front(front);
				lppath.push_front(front);
				if(front.parent==NULL)
					break;
				front=CLOSED[front.parent];
			}
			break;
		}
		for(int i=0; i<num_moves; ++i){
			int ii=front.i+diri[i];
			int jj=front.j+dirj[i];
			if(valid_move(front.i, front.j, ii, jj, map, options) && !vis[ii][jj]){
				sresult.nodescreated++;
				double cost=hypot(ii-front.i, jj-front.j);
				double estimated_cost=EstimateCost(ii, jj, ndi, ndj, options);
				q1.push(Node(ii, jj, front.g+cost+estimated_cost*options.hweight, front.g+cost, estimated_cost, cnt));
			}
		}
	}
	sresult.time=1.0*clock()/CLOCKS_PER_SEC-start_time;
}
void Search::go_DIJKSTRA(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
	int sti=map.getStart_i();
 	int stj=map.getStart_j();
 	int ndi=map.getGoal_i();
 	int ndj=map.getGoal_j();
 	h=map.getMapHeight();
 	w=map.getMapWidth();
  Node top=Node(sti, stj, EstimateCost(sti, stj, ndi, ndj, options)*options.hweight, 0, EstimateCost(sti, stj, ndi, ndj, options), NULL);
  q.push(top);
  int num_moves;
  if(options.allowdiagonal){
  	num_moves=8;
  }
  else{
  	num_moves=4;
  }
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
		if(vis[top.i][top.j])continue;
		vis[top.i][top.j]=1;
		CLOSED.push_back(top);
		++cnt;
		if(top.i==ndi && top.j==ndj){
			sresult.pathfound=true;
			sresult.pathlength=top.g;
			while(1){
				hppath.push_front(top);
				lppath.push_front(top);
				if(top.parent==NULL)
					break;
				top=CLOSED[top.parent];
			}
			break;
		}
		for(int i=0; i<num_moves; ++i){
			int ii=top.i+diri[i];
			int jj=top.j+dirj[i];
			if(valid_move(top.i, top.j, ii, jj, map, options) && !vis[ii][jj]){
				double cost=hypot(ii-top.i, jj-top.j);
				double estimated_cost=EstimateCost(ii, jj, ndi, ndj, options);
				q.push(Node(ii, jj, top.g+cost+estimated_cost*options.hweight, top.g+cost, estimated_cost, cnt));
			}
		}
	}
	sresult.nodescreated=q.size()+CLOSED.size();
	sresult.numberofsteps=CLOSED.size();
	sresult.time=1.0*clock()/CLOCKS_PER_SEC-start_time;
}
void Search::go_ASTAR(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
	int sti=map.getStart_i();
 	int stj=map.getStart_j();
 	int ndi=map.getGoal_i();
 	int ndj=map.getGoal_j();
 	h=map.getMapHeight();
 	w=map.getMapWidth();
  Node top=Node(sti, stj, EstimateCost(sti, stj, ndi, ndj, options)*options.hweight, 0, EstimateCost(sti, stj, ndi, ndj, options), NULL);
  q.push(top);
  int num_moves;
  if(options.allowdiagonal){
  	num_moves=8;
  }
  else{
  	num_moves=4;
  }
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
		if(vis[top.i][top.j])continue;
		vis[top.i][top.j]=1;
		CLOSED.push_back(top);
		++cnt;
		if(top.i==ndi && top.j==ndj){
			sresult.pathfound=true;
			sresult.pathlength=top.g;
			while(1){
				hppath.push_front(top);
				lppath.push_front(top);
				if(top.parent==NULL)
					break;
				top=CLOSED[top.parent];
			}
			break;
		}
		for(int i=0; i<num_moves; ++i){
			int ii=top.i+diri[i];
			int jj=top.j+dirj[i];
			if(valid_move(top.i, top.j, ii, jj, map, options) && !vis[ii][jj]){
				double cost=hypot(ii-top.i, jj-top.j);
				double estimated_cost=EstimateCost(ii, jj, ndi, ndj, options);
				q.push(Node(ii, jj, top.g+cost+estimated_cost*options.hweight, top.g+cost, estimated_cost, cnt));
			}
		}
	}
	sresult.nodescreated=q.size()+CLOSED.size();
	sresult.numberofsteps=CLOSED.size();
	sresult.time=1.0*clock()/CLOCKS_PER_SEC-start_time;
}

SearchResult Search::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options)
{
	ST = options.searchtype;
	BT = options.breakingties;
	if(options.searchtype == CN_SP_ST_BFS){
		go_BFS(Logger, map, options);
	}
	else if(options.searchtype == CN_SP_ST_DIJK){
		go_DIJKSTRA(Logger, map, options);
	}
	else if(options.searchtype == CN_SP_ST_ASTAR){
		go_ASTAR(Logger, map, options);
	}
	for(auto it1=lppath.begin(); it1!=lppath.end();){
		auto it2=it1;
		++it2;
		if(it2==lppath.end()){
			break;
		}
		auto it3=it2;
		++it3;
		if(it3==lppath.end()){
			break;
		}

		int dri1=(it3->i-it2->i); if(dri1) dri1/=abs(it3->i-it2->i);
		int dri2=(it2->i-it1->i); if(dri2) dri2/=abs(it2->i-it1->i);
		int drj1=(it3->j-it2->j); if(drj1) drj1/=abs(it3->j-it2->j);
		int drj2=(it2->j-it1->j); if(drj2) drj2/=abs(it2->j-it1->j);
		if((dri1==dri2) && (drj1==drj2)){
			lppath.erase(it2);
		}
		else{
			++it1;
		}
	}
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
