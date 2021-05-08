#include <bits/stdc++.h>
#include <ctime>
#include "carpooling.h"
#include <chrono>

using namespace std;

// #define unordered_map map
// #define unordered_set set

char * timeSourceName;
char * txtName;
string routeUsed,assignUsed;

/* nodes global structure */
vector< pair<double, double> > nodesToLatLon;
vector<long long int> nodeID;
unordered_map<long long int, int> idToNode;

/* Edges global structure */
vector< vector<long long int> > edges;
vector< vector<double> > edgeWeight;
vector< vector<long long int> > edgesReverse;
vector< vector<double> > edgeWeightReverse;
vector< vector<double> > edgeTime;

vector< unordered_map<int, vector<request> > > sourceTimeDestination; 
vector< unordered_map<int, vector<request> > > passengerRequest;
set<int> weekdays;
int cabsShared = 0;
int globalPickups = 0, globalRequests = 0, globalRejections = 0;  
long long int passengerWaitTime[MAX_CAB_CAPACITY+1][MAX_END_TIME];
double globalDistTravelled[MAX_CAB_CAPACITY+1];
int countFalse = 0;
int assignPickups = 0;
int maxWaitedSlot = MINUTE_BEFORE_REJECTION/DELTA_TIME;
int queueTimeSlot = 0, remainingQueueSize = 0;
int total_count=0;
double percentageDiff=0;

vector< passenger > passengerPickedList;
// At source v - a list of passenger waiting for cab
unordered_map<int, vector< passenger > > passengerQueue;

/* Time analysis */
long long cnt_bestpath = 0, cnt_bestpath2 = 0;
double total_time_bestpath = 0, total_time_bestpath2 = 0;
clock_t clockStartTime[5];

void start_clock(int slot = 0) {
	clockStartTime[slot] = clock();
}

double get_runtime(int slot = 0) {
	return float( clock()- clockStartTime[slot] )/ CLOCKS_PER_SEC;
}

bool isReachable(int source, int destination) {
	vector< double > distanceFromSource( nodeID.size() );
	vector<int> path;
	path = dijkstra_lengths(nodeID.size(), source, destination, distanceFromSource, edges, edgeWeight);
	if(distanceFromSource[destination] == MAX_DISTANCE || !distanceFromSource[ destination ] ) {
		printf("Not reachable -\tSource: %d\tDestination: %d\n", source, destination);
		return false;
	}
	return true;
}


void takeGraphInput(char * locationInputName,char * edgeInputName, string location) {
	ifstream Location;
	Location.open( locationInputName );
	string s;
	int index = 0, numNodes = 0, numEdge = 0;
	
	stringstream ss;
	char ch;
	if (location.compare("BJ") == 0) {
		getline(Location, s);
		ss.str( std::string() );
		ss.clear();
		ss<<s;
		ss>>numNodes>>ch>>numEdge;
	}

	while( getline(Location, s) ) {
		ss.str( std::string() );
		ss.clear();
		ss<<s;
		long long int id; double lon; double lat; 
		ss>>id>>ch>>lat>>ch>>lon; 
		nodesToLatLon.push_back( make_pair( lat, lon) );
		nodeID.push_back( id );
		idToNode[ id ] = index;
		
		index++;
		if(location.compare("BJ") == 0 && nodesToLatLon.size() == numNodes)
			break;
	}

	if (location.compare("SF") == 0 || location.compare("NY") == 0 || location.compare("SG") == 0) {
		Location.close();
		Location.open( edgeInputName );
	}

	cout<<"# of nodes= "<<nodeID.size()<<endl;
	// Get edges
	int count = 0;
	edges.resize(nodeID.size());
	edgeWeight.resize(nodeID.size());
	edgeTime.resize(nodeID.size());
	edgesReverse.resize(nodeID.size());
	edgeWeightReverse.resize(nodeID.size());
	while( getline(Location, s) ) {
		ss.str( std::string() );
		ss.clear();
		ss<<s;
		long long int numRandom; long long int node1; long long int node2; double weight; char ch; int oneWay = 1; double timeNeeded;
		if (location.compare("NY") == 0 || location.compare("BJ") == 0) {
			ss>>node1>>ch>>node2>>ch>>weight>>ch>>oneWay>>ch>>timeNeeded;
		}
		else {
			ss>>node1>>ch>>node2>>ch>>weight>>ch>>oneWay;
		}
		node1 = idToNode[node1];
		node2 = idToNode[node2];
		if (location.compare("NY") == 0 || location.compare("SG") == 0 || location.compare("SF") == 0) {
			weight /= 1000;
		}
		
		// printf("edge %lld %lld\n",node1,node2);
		edges[ node1 ].push_back( node2 );
		edgeWeight[ node1 ].push_back( weight );
		// edgeTime[ node1 ].push_back( timeNeeded );
		edgeTime[ node1 ].push_back( weight / SPEED );
		edgesReverse[ node2 ].push_back( node1 );
		edgeWeightReverse[ node2 ].push_back( weight );
		count = oneWay ? count +1: count+2;
		if( !oneWay ) {
			long long int temp = node1; node1 = node2; node2 = temp;
			edges[ node1 ].push_back( node2 );
			edgeWeight[ node1 ].push_back( weight );
			// edgeTime[ node1 ].push_back( timeNeeded );
			edgeTime[ node1 ].push_back( weight / SPEED );

			edgesReverse[ node2 ].push_back( node1 );
			edgeWeightReverse[ node2 ].push_back( weight );
		}

	}
	cout<<count+1<<endl;
	Location.close();
	cout<<"finished loading graph"<<endl;
	return ;
}


vector<int> getBestPath(int source, vector<int> destinations, int timeSlot, double defaultAlpha, vector<double> &actualTravelDists, vector<double> &shortestPathDistances) {
	printf("Trip start for %d passengers\n", (int)destinations.size());

	printf("Source: %d(%lld)\tDestination:",source,nodeID[source]);
	for (int i=0; i<destinations.size(); i++) {
		printf(" %d(%lld)",destinations[i],nodeID[destinations[i]]);
	}
	printf("\tTime slot: %d\n",timeSlot); fflush(stdout);

	start_clock();

	int firstDestination = destinations[0];

	vector< double > distanceFromSource( nodeID.size() );
	vector<vector< double > > distanceFromDestination( destinations.size() );
	vector<vector< double > > distanceToDestination( destinations.size() );

	vector<int> dagExPath;
	dagExPath = dijkstra_lengths(nodeID.size(), source, firstDestination, distanceFromSource, edges, edgeWeight);

	if(distanceFromSource[firstDestination] == MAX_DISTANCE || !distanceFromSource[ firstDestination ] ) {
		printf("Cannot reach destination?\n");
		vector<int> emptyPath;
		return emptyPath;
	}

	for (int i=0; i<destinations.size(); i++) {
		distanceToDestination[i].resize( nodeID.size() );
		distanceFromDestination[i].resize( nodeID.size() );
		dijkstra_lengths(nodeID.size(), destinations[i], source, distanceToDestination[i], edgesReverse, edgeWeightReverse);
		dijkstra_lengths(nodeID.size(), destinations[i], source, distanceFromDestination[i], edges, edgeWeight);
	}

	if (ROUTE == 1) {
		printf("  Start DAG\n"); fflush(stdout);
		pair<double, vector<int> > pp;
		auto startDex = std::chrono::high_resolution_clock::now(); 
		pp = findDAGPath( nodeID.size(), source, destinations, timeSlot/MINUTE_PER_HISTORY_SLOT, defaultAlpha, actualTravelDists, shortestPathDistances,
			sourceTimeDestination, distanceFromSource, distanceToDestination, distanceFromDestination, edges, edgeWeight);
		
		auto stopDex = std::chrono::high_resolution_clock::now();
		auto durationDex = std::chrono::duration_cast<std::chrono::microseconds>(stopDex - startDex);

		double dagScore= pp.first;
		dagExPath= pp.second;

		if( RUN_OPTIMAL && shortestPathDistances[0] < 2 && dagScore!= -1 )
		{
			cout<<endl;cout<<endl;cout<<endl;cout<<endl;cout<<endl;cout<<endl;cout<<endl;

			auto start = std::chrono::high_resolution_clock::now(); 

			pair<double, vector<int> > optimal = optimalScore( nodeID.size(), source, destinations, timeSlot/MINUTE_PER_HISTORY_SLOT, defaultAlpha, actualTravelDists, shortestPathDistances,
				sourceTimeDestination, edges, edgeWeight, distanceFromSource, distanceToDestination, distanceFromDestination );
			auto stop = std::chrono::high_resolution_clock::now(); 
			auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start); 

			vector<int> optimalPath= optimal.second;
			double optScore= optimal.first;

			cout<<"edges siz= "<<edges.size()<<endl;
			for(int pa=0;pa<optimalPath.size();pa++)
			{
				cout<<optimalPath[pa]<<" ";
			}
			if(optScore > 0)
			{

				total_count++;
				percentageDiff += (optScore-dagScore)/optScore;
				//updateTime((double)duration.count(), (double)durationDex.count(), shortestPathDistances[0]);
				cout<<endl;
				cout<<endl;cout<<endl;cout<<endl;cout<<endl;cout<<endl;cout<<endl;cout<<endl;cout<<endl;cout<<endl;cout<<endl;cout<<endl;cout<<endl;
				cout<<percentageDiff/total_count<<endl;
			}
		}

	}

	if (ROUTE == 2) {
		printf("  Start DAG EX\n"); fflush(stdout);
		pair<double, vector<int> > pp;

		cout<<"actDist= "<<actualTravelDists[0]<<endl<<endl;
		auto startDex = std::chrono::high_resolution_clock::now(); 

		pp = findDAGExtendedPath( nodeID.size(), source, destinations, timeSlot/MINUTE_PER_HISTORY_SLOT, defaultAlpha, actualTravelDists, shortestPathDistances,
			maxDepth, sourceTimeDestination, distanceFromSource, distanceToDestination, distanceFromDestination, edges, edgeWeight);
		auto stopDex = std::chrono::high_resolution_clock::now();
		auto durationDex = std::chrono::duration_cast<std::chrono::microseconds>(stopDex - startDex); 

		dagExPath = pp.second;
		double dexScore= pp.first;

		if( RUN_OPTIMAL && shortestPathDistances[0] < 3 && dexScore!= -1 )
		{
			cout<<endl;cout<<endl;cout<<endl;cout<<endl;cout<<endl;cout<<endl;cout<<endl;
			auto start = std::chrono::high_resolution_clock::now(); 
			
			pair<double, vector<int> > optimal = optimalScore( nodeID.size(), source, destinations, timeSlot/MINUTE_PER_HISTORY_SLOT, defaultAlpha, actualTravelDists, shortestPathDistances,
				sourceTimeDestination, edges, edgeWeight, distanceFromSource, distanceToDestination, distanceFromDestination );
			
			auto stop = std::chrono::high_resolution_clock::now(); 
			auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start); 

			vector<int> optimalPath = optimal.second;
			double optScore = optimal.first;

			//cout<<"edges siz= "<<edges.size()<<endl;
			for(int pa=0;pa<optimalPath.size();pa++)
			{
				cout<<optimalPath[pa]<<" ";
			}

			if(optScore > 0)
			{
				total_count++;
				percentageDiff += (optScore-dexScore)/optScore;
				//updateTime((double)duration.count(), (double)durationDex.count(), shortestPathDistances[0]);
			}
		}
	}

	if(ROUTE == 3 )
	{
		printf("  Start Optimal\n"); fflush(stdout);
		pair<double, vector<int> > pp;


		pp = findDAGPath( nodeID.size(), source, destinations, timeSlot/MINUTE_PER_HISTORY_SLOT, defaultAlpha, actualTravelDists, shortestPathDistances,
			 sourceTimeDestination, distanceFromSource, distanceToDestination, distanceFromDestination, edges, edgeWeight);

		dagExPath= pp.second;

		for(int pa=0;pa<dagExPath.size();pa++)
		{
			cout<<dagExPath[pa]<<" ";
		}
		cout<<endl;

		if(shortestPathDistances[0] < 3)
		{
			cout<<endl;cout<<endl;cout<<endl;cout<<endl;cout<<endl;cout<<endl;cout<<endl;

			pair<double, vector<int> > optimal  = optimalScore( nodeID.size(), source, destinations, timeSlot/MINUTE_PER_HISTORY_SLOT, defaultAlpha, actualTravelDists, shortestPathDistances,
				sourceTimeDestination, edges, edgeWeight, distanceFromSource, distanceToDestination, distanceFromDestination );
			
			vector<int> optimalPath = optimal.second;
			double optScore =  optimal.first;
			cout<<"edges siz= "<<edges.size()<<endl;
			for(int pa=0;pa<optimalPath.size();pa++)
			{
				cout<<optimalPath[pa]<<" ";
			}
			cout<<endl;
			cout<<endl;cout<<endl;cout<<endl;cout<<endl;cout<<endl;cout<<endl;cout<<endl;cout<<endl;cout<<endl;cout<<endl;cout<<endl;cout<<endl;
		}

	}

	cnt_bestpath++;
	total_time_bestpath += get_runtime();
	cout<<"Trip Found"<<endl; fflush(stdout);
	
	// cout<<"CHECK ALPHA VIOLATION AMOUNT: "<<checkAlphaPath( dagExPath, edges, edgeWeight, distanceFromSource[ firstDestination ])<<endl;

	return dagExPath;
}

int main(int argc, char const *argv	[])
{
	takeGraphInput("ny_location","ny_edge_time","NY");
}
