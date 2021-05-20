#pragma once
#include <bits/stdc++.h>
#include "ioD.h"
#include "utils.h"
using namespace std;

void get_all_trips_fast(int source,int start_time,vector<request> &trips,
	vector< unordered_map<int, vector<request> > > &dataset ) {

	if( source < dataset.size() ) {
		if( dataset[source].find(start_time) != dataset[source].end() ) {
			trips = dataset[source][ start_time ];
		}
	}
}

vector<vector<pair<int, double> > > nearbyNodes;

void generateNearbyNodes(int N, const vector< vector<long long int> > &edges, const vector< vector<double> > &edgeWeight) { 

	vector<map<int, double> > checker(N);
	nearbyNodes.resize(N);

	int cntNearby = 0;

	// int visited[ dag.size() ];
	double distancesFromNode[ N ];
	for(int i=0;i<N;i++)
	{
		int node = i;
		for(int j=0;j<N;j++)
		{
			distancesFromNode[j]=MAX_DISTANCE;
		}
		distancesFromNode[ node ]=0;
		priority_queue<pair<double, int> > pq;
		pq.push(make_pair(0, node));
		// visited[ nodeToDagIndex[ node ] ] =1;

		while( !pq.empty() ) 
		{ 
			pair<double, int> x = pq.top(); 
			pq.pop();
			int u = x.second;

			if( distancesFromNode[ u ] > DELTA_DETOUR )
				break;

			if(checker[ node ].find(u) == checker[i].end() )
			{
				checker[ node ][ u ] = distancesFromNode[ u ];
				nearbyNodes[ node ].push_back(make_pair(u , distancesFromNode[ u ]));
				cntNearby++;
			}

			for(int j = 0; j < edges[ u ].size(); j++) { 	
				int v = edges[ u ][ j ];
				double alt = distancesFromNode[ u ] + edgeWeight[ u ][ j ];
				if( alt < distancesFromNode[ v ] )
				{ 	
					distancesFromNode[ v ] = alt;
					pq.push( make_pair( -alt, v) );
				}
			}
		}

	}

	printf("Finish generating nearby nodes (%d)\n",cntNearby); fflush(stdout);
}

double getClusterScore(int v, vector<int> &destinations, int start_time, double defaultAlpha, vector<double> &distancesTravelled, double newDist, vector< double > &shortestPathDistances,
	vector< unordered_map<int, vector<request> > > &dataset, const vector<vector< double > > &distanceToDestination, const vector<vector< double> > &distanceFromDestination) {	

	double scoreReturn=0;
	for(int i=0;i<=nearbyNodes[ v ].size();i++)
	{
		if(destinations.size()==1)
		{
			int u;
			double uDist;
			vector<request> trips;

			//printf("getClusterScore: nearbyNodes[v].size() is %d\n",nearbyNodes[v].size());

			if(i == nearbyNodes[v].size())
			{
				u= v;
				uDist=0;
				get_all_trips_fast(u, start_time, trips, dataset);
			}
			else
			{
				pair<int, double> p = nearbyNodes[ v ][ i ];
				u = p.first;
				// u is the middle point
				double uDist = p.second;
				double timeOffset=0;

				if(TIME_VARYING)
					timeOffset = (uDist/SPEED)/MINUTE_PER_HISTORY_SLOT;	

				get_all_trips_fast(u, start_time + timeOffset, trips, dataset);

			}

			double shortestPath = shortestPathDistances[0];
			double allowed= defaultAlpha * shortestPath;

			double distTillU = distancesTravelled[ 0 ] + newDist + uDist;
			double sigmScore=0;
			double sigmoidConst = 0.1;
			double origDist=0;
			origDist = distanceToDestination[ 0 ][ v ];

			for(int j = 0; j < trips.size(); j++) 
			{
				int dest2 = trips[ j ].destination;
				double revenue = trips[ j ].revenue;

				// dest2 = destination of the cab at midpoint  
				double uToDest2 = query(u, dest2);
				double dest2ToDest1 = distanceToDestination[ 0 ][ dest2 ];
				double dest1ToDest2 = distanceFromDestination[ 0 ][ dest2 ];
				double deltaDist = 1e10;


				if( uToDest2 + dest2ToDest1 + distTillU <= allowed  ) {
					deltaDist = min(deltaDist, uDist+ uToDest2 + dest2ToDest1);

				}

				double uToDest1 = distanceToDestination[ 0 ][ u ];
				double distanceA = ( distTillU + uToDest1 + dest1ToDest2);
				double distanceB = defaultAlpha * uToDest2;
				if( distanceA <= distanceB && distanceA <= allowed) {
					deltaDist = min(deltaDist, uDist + uToDest1 + dest1ToDest2);
				}

				// This trip is not feasible
				if (deltaDist > 1e9) continue;

				scoreReturn += 1;
				// double profit = ( tsd + uToDest1 ) - FUEL * ( distanceTravelled + uToDest2 + dest1ToDest2 );
				//double profit = revenue - FUEL * (deltaDist-origDist);
				// use direct e^x instead of that
				//sigmScore += exp(sigmoidConst * profit);
			}
			// cout<<"sigmScore= "<<sigmScore<<endl;
			//scoreReturn+= exp(-uDist) * sigmScore;

		}

		if(destinations.size() == 2)
		{
			int u;
			double uDist;
			vector<request> trips;
			// cout<<"arre"<<endl;

			if(i == nearbyNodes[v].size())
			{
				u= v;
				uDist=0;  
				get_all_trips_fast(u, start_time, trips, dataset);
			}
			else
			{
				pair<int, double> p = nearbyNodes[ v ][ i ];
				u = p.first;
				// u is the middle point
				uDist = p.second;	
				double timeOffset=0;

				if(TIME_VARYING)
					timeOffset = (uDist/SPEED)/MINUTE_PER_HISTORY_SLOT;	

				get_all_trips_fast(u, start_time + timeOffset, trips, dataset);

			}

			int d1 = destinations[0];
			int d2 = destinations[1];
			double shortestPath1 = shortestPathDistances[0];
			double shortestPath2 = shortestPathDistances[1];

			// double allowed= defaultAlpha * shortestPathDistances;

			double distTillU1 = distancesTravelled[ 0 ] + newDist + uDist;
			double distTillU2 = distancesTravelled[ 1 ] + newDist + uDist;

			double sigmScore=0;
			double sigmoidConst = 0.1;			

			for(int j = 0; j < trips.size(); j++) 
			{
				int d3 = trips[ j ].destination;
				double revenue = trips[ j ].revenue;

				// dest2 = destination of the cab at midpoint  
				double shortestPath3 = query(u, d3);

				double dest3ToDest1 = distanceToDestination[ 0 ][ d3 ];
				double dest1ToDest3 = distanceFromDestination[ 0 ][ d3 ];
				double dest3ToDest2 = distanceToDestination[ 1 ][ d3 ];
				double dest2ToDest3 = distanceFromDestination[ 1 ][ d3 ];
				double dest2ToDest1 = distanceToDestination[ 0 ][ d2 ];
				double dest1ToDest2 = distanceFromDestination[ 0 ][ d2 ];

				double uToDest1 = distanceToDestination[ 0 ][ u ];
				double uToDest2 = distanceFromDestination[ 1 ][ u ];
				double uToDest3 = shortestPath3;
				
				double origDist=0;
				origDist = distanceToDestination[ 0 ][ v ] + dest1ToDest2;

				double deltaDist = 1e10;

				double allowed1 = defaultAlpha * shortestPath1;
				double allowed2 = defaultAlpha * shortestPath2;
				double allowed3 = defaultAlpha * shortestPath3;
				// see for permutations


				// 123
				if((distTillU1 + uToDest1 <=allowed1) && (distTillU2 + uToDest1 + dest1ToDest2 <=allowed2) && (uToDest1 + dest1ToDest2 + dest2ToDest3 <= allowed3))
				{
					deltaDist = min(deltaDist, uDist + uToDest1 + dest1ToDest2 + dest2ToDest3);
				}
				// 132
				if((distTillU1 + uToDest1 <=allowed1) && (distTillU2 + uToDest1 + dest1ToDest3 + dest3ToDest2 <=allowed2) && (uToDest1 + dest1ToDest3 <= allowed3))
				{
					deltaDist = min(deltaDist, uDist + uToDest1 + dest1ToDest3 + dest3ToDest2);
				}
				// 213
				if((distTillU1 + uToDest2 + dest2ToDest1 <=allowed1) && (distTillU2 + uToDest2 <=allowed2) && (uToDest2 + dest2ToDest1 + dest1ToDest3 <= allowed3))
				{
					deltaDist = min(deltaDist, uDist + uToDest2 + dest2ToDest1 + dest1ToDest3);
				}
				// 231
				if((distTillU1 + uToDest2 + dest2ToDest3 + dest3ToDest1 <=allowed1) && (distTillU2 + uToDest2 <=allowed2) && (uToDest2 + dest2ToDest3 <= allowed3))
				{
					deltaDist = min(deltaDist, uDist + uToDest2 + dest2ToDest3 + dest3ToDest1);
				}
				// 312
				if((distTillU1 + uToDest3 + dest3ToDest1 <=allowed1) && (distTillU2 + uToDest3 + dest3ToDest1 + dest1ToDest2 <=allowed2) && (uToDest3 <= allowed3))
				{
					deltaDist = min(deltaDist, uDist + uToDest3 + dest3ToDest1 + dest1ToDest2);
				}
				// 321
				if((distTillU1 + uToDest3 + dest3ToDest2 + dest2ToDest1 <=allowed1) && (distTillU2 + uToDest3 + dest3ToDest2 <=allowed2) && (uToDest3 <= allowed3))
				{
					deltaDist = min(deltaDist, uDist + uToDest3 + dest3ToDest2 + dest2ToDest1);
				}



				// This trip is not feasible
				if (deltaDist > 1e9) continue;

				scoreReturn += 1;
				// double profit = ( tsd + uToDest1 ) - FUEL * ( distanceTravelled + uToD3 + dest1ToDest2 );
				//double profit = revenue - FUEL * (deltaDist-origDist);
				// use direct e^x instead of that
				//sigmScore += exp(sigmoidConst * profit);
			}
			//scoreReturn+= exp(-uDist) * sigmScore;


		}

	}	

	if(scoreReturn==0) {
		// printf("Get Cluster Score at %d = %.2f\n",v, scoreReturn); fflush(stdout);
		return 0;
	}

	// printf("Before log %d = %.2f... ",v, scoreReturn); fflush(stdout);
	//scoreReturn= log(1+ scoreReturn);
	// printf("Get Cluster Score at %d = %.2f\n",v, scoreReturn); fflush(stdout);
	return scoreReturn;
	
}


double get_expected_trips(int v, vector<int> &d, int start_time, double alpha, vector<double> &distancesTravelled, double newDist, vector< double > &shortestPathDistances,
	vector< unordered_map<int, vector<request> > > &dataset, const vector<vector< double > > &distanceToDestination, const vector<vector< double > > &distanceFromDestination) {  	

	double expConst = 1.05;

	vector<request> tripList;
	get_all_trips_fast(v, start_time, tripList, dataset);

	if (tripList.size() == 0) {
		return 0;
	}

	// triplist has all the destinations
	int counter = 0;

	// calculating compatibility distance for the nodes in the dataset; counter returns it
	if (d.size() == 1) {
		double tvd, tsv;
		tsv = distancesTravelled[ 0 ] + newDist;
		tvd = distanceToDestination[ 0 ][ v ];

		for(int i = 0; i < tripList.size(); i++) {

			int w = tripList[ i ].destination;
			double tvw = query(v, w);
			double tdw = distanceFromDestination[ 0 ][ w ];

			if( tvd + tdw <= alpha*tvw ) {
				counter += 1; continue;
			}
			
			double twd = distanceToDestination[ 0 ][ w ];
			double distanceA = ( tsv + tvw + twd );
			double distanceB = alpha*shortestPathDistances[ 0 ];
			if( distanceA <= distanceB) {
				counter += 1; continue;
			}
		}

	}
	
	if (d.size() == 2) {

		double ts0v = distancesTravelled[ 0 ] + newDist;
		double ts1v = distancesTravelled[ 1 ] + newDist;

		double tvd0 = distanceToDestination[ 0 ][ v ];
		double tvd1 = distanceToDestination[ 1 ][ v ];

		double ts0d0 = shortestPathDistances[ 0 ];
		double ts1d1 = shortestPathDistances[ 1 ];

		double td0d1 = distanceFromDestination[ 0 ][ d[1] ];
		double td1d0 = distanceFromDestination[ 1 ][ d[0] ];

		for(int i = 0; i < tripList.size(); i++) {
			int w = tripList[ i ].destination;		

			double tvw = query(v, w);
			double td0w = distanceFromDestination[ 0 ][ w ];
			double twd0 = distanceToDestination[ 0 ][ w ];
			double td1w = distanceFromDestination[ 1 ][ w ];
			double twd1 = distanceToDestination[ 1 ][ w ];

			if( ( ts0v + tvd0 <= alpha*ts0d0) && ( ts1v + tvd0 + td0d1 <= alpha*ts1d1) && ( tvd0 + td0d1 + td1w <= alpha*tvw )  ) {
				counter += 1; continue;
			}
			if( ( ts0v + tvd0 <= alpha*ts0d0) && (tvd0 + td0w <= alpha*tvw) &&   (ts1v + tvd0 + td0w + twd1 <= alpha*ts1d1 ) ) {
				counter += 1; continue;
			}
			if ( (ts1v + tvd1 <= alpha*ts1d1) && (ts0v + tvd1 + td1d0 <= alpha*ts0d0) && ( tvd1 + td1d0 + td0w <= alpha*tvw ) ) {
				counter += 1; continue;
			}
			if ( (ts1v + tvd1 <= alpha*ts1d1) && (tvd1 + td1w <= alpha*tvw) &&  ( ts0v + tvd1 + td1w +twd0  <= alpha*ts0d0 ) ) {
				counter += 1; continue;
			}
			if( ( ts0v + tvw + twd0 <= alpha*ts0d0 ) && ( ts1v + tvw + twd0 + td0d1 <= alpha*ts1d1 ) ) {
				counter += 1; continue;
			}
			if( ( ts1v + tvw + twd1 <= alpha*ts1d1) && ( ts0v + tvw + twd1 +td1d0 ) <= alpha*ts0d0 ) {
				counter += 1; continue;
			}
		}
	}

	// printf("  %.4f - %.4f * %d = %.4f\n",newDist, pow(expConst, -newDist), counter, pow(expConst, -newDist) * counter);
	if(USE_DECAY)
		return pow(expConst, -newDist) * counter;
	else
		return counter;
	// return counter;
}

double get_expected_trips_mock() {
	return 1.0;
}