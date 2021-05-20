#pragma once
#include <bits/stdc++.h>
#include "demand_prediction.h"
#include "utils.h"
using namespace std;

double getPathDist(vector<int> &path, const vector< vector< long long int> > &edges, const vector< vector<double> > &edgeWeight) {
	double cumDist = 0;
	for (int i=0; i<path.size(); i++) {
		int v = path[i];
		if (i > 0) {
			int u = path[i-1];
			for (int j=0; j<edgeWeight[u].size(); j++) {
				if (edges[u][j] == v) {
					cumDist += edgeWeight[u][j];
					break;
				}
			}
		}
	}
	return cumDist;
}

/*
Dijkstra algorithm
Given (S,D), return the shortest path
*/
vector<int> dijkstra_lengths(int N, int S, int D, vector< double > &distanceFromSource,
	const vector< vector<long long int> > &edges, const vector< vector<double> > &edgeWeight) { 

	vector<int> prevNode(N);
	for(int i = 0; i < N; i++)
	{ 	distanceFromSource[ i ] = MAX_DISTANCE;
		prevNode[ i ] = -1;
	}

	distanceFromSource[ S ] = 0;	
	priority_queue< pair<float, int> > dj;
	dj.push( make_pair(0, S) );
	
	pair<float, int> x;
	int u, v;
	float alt;

	while( dj.size() ) 
	{ 
		x = dj.top(); 
		dj.pop();
		u = x.second;

		if( distanceFromSource[ u ] >= MAX_DISTANCE )
			break;

		for(int i = 0; i < edges[ u ].size(); i++) { 	
			v = edges[ u ][ i ];
			alt = distanceFromSource[ u ] + edgeWeight[ u ][ i ];
			if( alt < distanceFromSource[ v ] )
			{ 	distanceFromSource[ v ] = alt;
				dj.push( make_pair( -alt, v) );
				prevNode[ v ] = u;
			}
		}
	}

	for(int i = 0; i < N; i++)
	{ 	if( distanceFromSource[ i ] >= MAX_DISTANCE ) {
			distanceFromSource[ i ] = MAX_DISTANCE;
			prevNode[ i ] = -1;
		}
	}

	vector<int> path;
	int node = D;
	while( true ) {
		path.push_back( node );
		if( ( node == S ) || ( prevNode[ node ] == -1) )
			break;
		node = prevNode[ node ];
	}
	
	reverse(path.begin(), path.end());

	return path;
}



/*
DAG extended score algorithm
Given (S,D), construct a DAG based on distance to destination, then perform a dynamic programming to obtain max score path. 
Back edges are allowed as long as the distance is less than the maxDepth parameter.
Cycle is not allowed.
For multi passenger cab, we always seek for planning the best route to the first destination
*/

vector< vector< pair<double, vector<int> > > > extendEdge;

void extendEdges(int source, int node, double maxDepth, vector<int> &path, vector< pair<double, vector<int> > > &paths,
	double pathLen, const vector< vector<long long int> > &edges, const vector< vector<double> > &edgeWeight) {
	
	for( int i = 0; i < path.size(); i++) {
		if( node == path[i] )
			return;
	}
	path.push_back( node );

	if( (pathLen >= maxDepth) && ( path.size() > 2 ) ) {
		path.pop_back();
		return ;
	}

	if( path.size() > 1 ) {
		paths.push_back( make_pair(pathLen, path) ) ;
	}
	
	for(int j=0; j < edges[ node ].size(); j++) {
		int newNode = edges[ node ][j];
		extendEdges( source, newNode, maxDepth, path, paths, pathLen + edgeWeight[ node ][ j ], edges, edgeWeight ) ;
	}
	//cout<<"node= "<<node<<endl;

	path.pop_back();
}

 
void assignExtendEdge(int N, double maxDepth, const vector< vector<long long int> > &edges, const vector< vector<double> > &edgeWeight) {
	extendEdge.resize(N);
	int cnt_extended = 0;
	for( int i = 0; i < N; i++) {
		vector<int> path;
		vector< pair< double, vector<int> > > paths;
		extendEdges(i, i, maxDepth, path, paths, 0, edges, edgeWeight);
		cnt_extended += paths.size();
		extendEdge[ i ] = paths;
	}
	printf("Extend edge assigned (Total = %d)\n",cnt_extended); fflush(stdout);
}

int short_outputs=0;

pair<double, vector<int> >findDAGExtendedPath(int N, int source, vector<int> destinations, int timeSlot, double defaultAlpha, vector<double> &actualTravelDists, vector<double> &shortestPathDistances, double maxDepth, vector< unordered_map<int, vector<request> > > &dataset,
	const vector< double > &distanceFromSource, const vector<vector< double > > &distanceToDestination, const vector<vector< double > > &distanceFromDestination, const vector< vector<long long int> > &edges, const vector< vector<double> > &edgeWeight) {

	int firstDestination = destinations[0];

	map<int, int> dummy;

	vector< vector< long long int> > extendEdgeWeights(N);
	for( int i = 0; i < N; i++) {
		extendEdgeWeights[i].resize(extendEdge[i].size());
	}

	vector< pair< double, int> > dag;

	double remainShortestDist = 0;
	bool alphaImpossible = false;
	for(int i=0; i < N ; i++) {
		remainShortestDist = 0;
		alphaImpossible = false;
		for (int j=0; j<destinations.size(); j++) {
			if (j) remainShortestDist += distanceToDestination[j][destinations[j-1]];
			if (actualTravelDists[j] + distanceFromSource[i] + distanceToDestination[0][i] + remainShortestDist > defaultAlpha * shortestPathDistances[j]) {
				alphaImpossible = true; break;
			}
		}
		
		if (!alphaImpossible) dag.push_back( make_pair(-distanceToDestination[0][ i ], i ) );
	}

	vector<int> path;
	if(dag.size() <= 1)
	{
		// This happens because query is not as accurate as Dijkstra
		vector <double> dummy(N);
		path= dijkstra_lengths(N, source, firstDestination, dummy, edges, edgeWeight);
		cout<<"shortestpaths used instead DAG" <<endl;
		return make_pair(-1,path);
	}

	sort( dag.begin(), dag.end() );

	// (Distance, (Last Node, Ex Edge used) ) at dag[i] given a particular score
	vector< vector <pair<double, double> > > scores( dag.size() );
	vector< vector< pair<vector<int> ,int > > > store( dag.size() );

	vector< int> nodeToDagIndex(N);
	vector< int> dagIndexToNode(N);
	for (int i=0; i<N; i++) {
		nodeToDagIndex[i] = -1;
		dagIndexToNode[i] = -1;
	}
	for(int i=0; i <  dag.size() ; i++) {
		nodeToDagIndex[ dag[i].second ] = i;
		dagIndexToNode[ i ] = dag[i].second;
	}
	// cout<<"start 4"<<endl;
	// fflush(stdout);
	
	int startIndex = nodeToDagIndex[ source ];
	int endIndex = nodeToDagIndex[ firstDestination ];
	
	double neginf = -1e10;
	int negint = INT_MIN;

	remainShortestDist = 0;
	double delta_dist = 1e10;
	for (int j=0; j<destinations.size(); j++) {
		if (j) remainShortestDist += distanceToDestination[j][destinations[j-1]];
		delta_dist = min(delta_dist, defaultAlpha * shortestPathDistances[j] - actualTravelDists[j] - remainShortestDist);
	}

	vector<int> tadd;
	tadd.push_back(negint);
	vector <pair<double, double> > forRow;
	vector <pair<vector<int>, int > > forstore;

	for(int j=0; j<=DISCRETE_TIME; j++)
	{
		 forRow.push_back( make_pair(neginf, neginf) );
		 forstore.push_back( make_pair(tadd, negint ));
	}
	for(int i=0; i<dag.size(); i++)
	{       
		scores[i] = forRow;
		store[i] = forstore;
	}

	scores[startIndex][ 0 ]=make_pair(0, 0);
	store[startIndex][0].first =tadd;
	store[startIndex][0].second =-1;

	for(int i = startIndex; i < dag.size(); i++) 
	{	
		for(int k=0;k<=DISCRETE_TIME;k++)
		{
			int u = dag[i].second;
			if(scores[i][k].first < 0)
				continue;
			// cout << i << " " << k << endl;
			for(int j = 0; j < extendEdge[u].size(); j++) 
			{
				pair<double, vector<int> > v = extendEdge[u][j];
				bool continueLoop = false;
				bool invalidNode = false;
				// bool reachAbleError = false;
				int pathSize = v.second.size();
				vector<int> ex_edge = v.second;

				if( pathSize < 1)
					continue;

				int lastNode =  ex_edge[ pathSize - 1 ];
				if(nodeToDagIndex[ lastNode ]<= i)
					continue;

				if ( lastNode == u )
					continue;

				// All nodes in the extended path except the last node has to be before u in the DAG
				for(int piter = 1; piter < pathSize ; piter++ ) 
				{
					int pathNode = ex_edge[ piter]; 
					
					if( ( nodeToDagIndex[ pathNode ] >= i )  ^  ( piter == ( pathSize -1 ) ) ) {
						continueLoop = true;
						break;
					}

					if( nodeToDagIndex[ pathNode ] == -1 ) {
						invalidNode = true;
						break;
					}
				}

				if( continueLoop || invalidNode )
					continue;
				map< int, int> markNodes;
				for(int k1 = 1; k1 < pathSize ; k1++ ) {
					markNodes[ ex_edge[ k1 ] ] = 1;
				} 

				int vIndex = nodeToDagIndex[ lastNode ];

				bool reachAbleError = false;
				int traceLoc = nodeToDagIndex[ u ];
				int trace_time = k;
				
				// bscktracking checking
				while ( (traceLoc != -1 ) && ( !reachAbleError ) ) 
				{
					// TODO: pass by reference
					pair<vector<int>, int > prev = store[ traceLoc ][ trace_time ];
					vector<int> prev_loc = prev.first;
					int prev_time = prev.second;
					// cout<<"huh4"<<endl;
					for(int loc = prev_loc.size()-1 ; loc>=0 ; loc-- )
					{
						if( loc >= 0 && markNodes.find( prev_loc[ loc ]  ) != markNodes.end() ) 
						{
							reachAbleError = true;
							break;
						}
					}

					// cout<<"traceLoc= "<<traceLoc<<endl;
					if (traceLoc != -1) {
						trace_time = prev_time; 
					}
					// cout<<"tracetime= "<<trace_time<<endl;
					if(prev_loc[0] < 0)
						break;
					else
						traceLoc = nodeToDagIndex [ prev_loc[0] ];

					// cout<<"traceLoc2= "<<traceLoc<<endl;
					if(traceLoc != -1)
						if( distanceToDestination[0][ prev_loc[ 0 ] ] - distanceToDestination[0][ lastNode ] > maxDepth ){
							break;
					
					}
				}

				if( reachAbleError && ( u != source ) && ( pathSize != 2 ) ) {
					continue;
				} 

				//everything is fine here 
				double total_nodescore = 0;
				double totalEdgeDist = 0;

				double prevNodeDist= scores[ i ][ k ].second;
				for(int pat = 1; pat < pathSize ; pat++ ) 
				{

					int pathNode = ex_edge[pat];
					double pres_edge = 0;

					for( int edgeIndex = 0; edgeIndex < edges[ ex_edge[ pat- 1 ] ].size(); edgeIndex++)
					{
						if( edges[ ex_edge[ pat - 1 ] ][ edgeIndex ] == ex_edge[ pat ] ) 
						{
							pres_edge = edgeWeight[ ex_edge[ pat - 1 ] ][ edgeIndex ];	
							break;
						}
					}
					totalEdgeDist += pres_edge;
					int offset= (DISCRETE_TIME * totalEdgeDist / delta_dist);
					double newDist = (prevNodeDist + totalEdgeDist);
					double timeOffset=0;
					if(TIME_VARYING)
					{
						timeOffset= ((int)(newDist/SPEED))/MINUTE_PER_HISTORY_SLOT;
					}
					double node_score = 0;

					if(USE_CLUSTER)
					{
						node_score = getClusterScore(pathNode, destinations, timeSlot + timeOffset, defaultAlpha, actualTravelDists, newDist, shortestPathDistances, 
						dataset, distanceToDestination, distanceFromDestination);
					}
					else
					{
						node_score = get_expected_trips(pathNode, destinations, timeSlot + timeOffset, defaultAlpha, actualTravelDists, newDist, shortestPathDistances, 
						dataset, distanceToDestination, distanceFromDestination);
					}
					// double node_score = getClusterScore(pathNode, destinations, timeSlot + ((int)(newDist/SPEED))/MINUTE_PER_HISTORY_SLOT, defaultAlpha, actualTravelDists, newDist, shortestPathDistances,
						// dataset, distanceToDestination, distanceFromDestination);
					total_nodescore += node_score;

				}
				// cout<<"huh3 "<<endl;

				double totaloffset1= (DISCRETE_TIME * totalEdgeDist / delta_dist);
				int newTime = (DISCRETE_TIME * (totalEdgeDist + prevNodeDist) / delta_dist);

				if(nodeToDagIndex[ lastNode ] > i && newTime <= DISCRETE_TIME)
				{
									
					if((scores[ nodeToDagIndex[ lastNode ] ][ newTime ].first  <  scores[ i ][ k ].first +  total_nodescore) || 
							((scores[ nodeToDagIndex[ lastNode ] ][ newTime ].first  ==  scores[ i ][ k ].first +  total_nodescore) && 
								scores[ i ][ k ].second + totalEdgeDist < scores[ nodeToDagIndex[ lastNode ] ][ newTime ].second))
					{
						scores[ nodeToDagIndex[ lastNode ] ][ newTime ].first = scores[ i ][ k ].first + total_nodescore;
						scores[ nodeToDagIndex[ lastNode ] ][ newTime ].second = scores[ i ][ k ].second + totalEdgeDist;
						store[ nodeToDagIndex[ lastNode ] ][ newTime ] = make_pair(ex_edge, k);
					}
				}					
			}
		}
		
	}

	// for(int i=0; i<dag.size(); i++)
 //    {
 //        // vector <double> fogRow;
 //        int flag=0;
 //        for(int j=0; j<=DISCRETE_TIME; j++)
 //        {
 //             if(scores[i][j].first>=0)
 //             {
 //             	cout<<"i= "<<i<<" j= "<<j<<" sc= ";
 //             	cout<<scores[i][j].first<<" ";
 //             	flag=1;
 //             }
 //        }
 //        // scores.push_back(fogRow);
 //        if(flag==1)
 //        	cout<<endl;
 //    }
	// backtracking -> needed to change just like updation
	double max_score=-1;
	// long long int maxindex=-1;
	int max_time=-1;
	for( int popp=0;popp<=DISCRETE_TIME;popp++) 
	{
		//printf(" -- Score = %d (%.4f < %.4f?)\n",it->first, it->second.first, distFromSourceToDestination*alpha);
		if( scores[endIndex][popp].first>max_score ) 
		{
			max_score= scores[endIndex][popp].first;
			// maxindex=it->second.second.first;
			max_time= popp;
		}
	}

	cout<<max_score<<endl;
	cout<<max_time<<endl;

	if(max_score==-1)
	{
		vector <double> dummy(N);
		path= dijkstra_lengths(N, source, firstDestination, dummy, edges, edgeWeight);
		short_outputs+=1;
		cout<<"findDAGExPath: shortestpaths= "<<short_outputs<<endl;
		return make_pair(-1,path);
	}
	

	int traceLocation = endIndex;
	int tracetime = max_time;

	while (traceLocation != -1) 
	{
		// printf(" At %d: Time = %d (score: %.4f)\n",traceLocation, tracetime, scores[ traceLocation ][tracetime].first); fflush(stdout);
		path.push_back(dagIndexToNode[ traceLocation] );
		int prevtime = store[ traceLocation ][ tracetime ].second;
		vector<int> prev_locs = store[ traceLocation ][tracetime].first;
		for(int loc = ((int)prev_locs.size())-2; loc>=1; loc--)
		{
			path.push_back(prev_locs[loc]);
		}
		traceLocation = prev_locs[0];

		if(traceLocation < 0)
		{
			break;
		}
		traceLocation=nodeToDagIndex[traceLocation];

		tracetime = prevtime;
	}
	reverse(path.begin(), path.end());

	cout<<"findDAGExPath: BEST SCORE: dex 0.3  "<<max_score<<endl;
	double pathlen=getPathDist(path, edges, edgeWeight);
	cout<<"findDAGExPath: Pathlen= "<<pathlen<<endl;
	cout<<"findDAGExPath: delta_dist= "<<delta_dist<<endl;
	// for(int i=0;i<path.size();i++ )
	// {
	// 	cout<<path[i]<<endl;
	// }
	double pathlen2=distanceFromSource[firstDestination];
	cout<<"findDAGExPath: disjkstra Pathlen= "<<pathlen2<<endl;

	printf("findDAGExPath: start: %d end: %d\n",startIndex, endIndex);
	/*
	printf("findDAGExPath: DAG index | node score\n");
	for(int i = 0; i < dag.size(); i++) {
		int u = dag[i].second;
		double node_score = get_expected_trips(u, destinations, timeSlot, defaultAlpha, actualTravelDists, 0, shortestPathDistances, 
					dataset, distanceToDestination, distanceFromDestination);
		printf("%d | %.4f\n",i,node_score);

	}
	printf("findDAGExPath: DAG index (u) | DAG index (v) | edge weight\n");
	for (int i = 0; i<dag.size(); i++) {
		int u = dag[i].second;
		for(int j = 0; j < edges[u].size(); j++) {
			int v = edges[u][j];
			if( nodeToDagIndex[ v ] == -1 )
				continue;
			//some nodes are adjacent to themselves in the graph
			if(u == v)
				continue;
			printf("%d | %d | %.4f\n",nodeToDagIndex[u], nodeToDagIndex[v],edgeWeight[ u ][ j ]);
		}
	}
	*/
	printf("findDAGExPath: Path (DAG index): ");
	for (int i=0; i<path.size(); i++) {
		printf(" %d",nodeToDagIndex[path[i]]);
	}
	printf("\n");
	printf("findDAGExPath: Path (nodes): ");
	for (int i=0; i<path.size(); i++) {
		printf(" %d",path[i]);
	}
	printf("\n");

	return make_pair(max_score, path);
}

