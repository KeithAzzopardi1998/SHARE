#pragma once
#include "ioD.h"

int DEBUG = 1;
const int DELTA_TIME = 1;
const int DISCRETE_TIME = 100;
const int MAX_DISTANCE = 1000000;
const bool TIME_VARYING = false;
const double SPEED = 1; // km per min
const int MINUTE_PER_HISTORY_SLOT = 15;
const bool USE_CLUSTER= true;
const double DELTA_DETOUR = 0.2;
const bool USE_DECAY= false;

struct request {
	int id;
	int timeSlot;
	int source;
	int destination;
	double revenue;
};

char * degName, * outName, *inName;
edgeL * deg;
edgeS * labelout, *labelin;
edgeS * labelx, * labely;

map< pair<int, int>, double > timeOptimize; 
map< int, map<int, double> > distanceFrequentNodes;
vector< bool > frequentPickup;
vector< bool > frequentDrop;

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

// Takes in node index
double query(int x, int y)
{	
	if (x == y) return 0;

	// if we already have this key pair value then return 
	if( frequentPickup[ x ] && frequentDrop[ y ] ) {
		return distanceFrequentNodes[ x ][ y ];
	}
	
	if( timeOptimize.find( make_pair(x, y) ) != timeOptimize.end() ) {
		return timeOptimize[ make_pair(x, y) ]; 
	}

	int xx = x, yy = y;

	x = ((deg[xx].x<<32)>>32);
	y = ((deg[yy].x<<32)>>32);
		
	if (x > y)
	{
		labelx = labelout + deg[xx].w;
		labely = labelin + deg[yy].y;
	}
	else
	{
		int xy = x; x = y; y = xy;
		labelx = labelin + deg[yy].y;
		labely = labelout + deg[xx].w;
	}

	int ans = 1000000, i = 0, j = 0;

	if (labelx[i].x != -1 && labely[j].x != -1)
	while (labelx[i].x < y)
	{
		if (labelx[i].x == labely[j].x) 
		{
			ans = ans>(labelx[i].w + labely[j].w)?(labelx[i].w + labely[j].w):ans;
			if (labelx[++i].x == -1) break;
			if (labely[++j].x == -1) break;
		}
		else if (labelx[i].x < labely[j].x)
		{
			if (labelx[++i].x == -1) break;
		}
		else if (labely[++j].x == -1) break;
	}
	
	while (labelx[i].x != -1 && labelx[i].x < y) i++;
	if (labelx[i].x == y) ans = ans>labelx[i].w?labelx[i].w:ans;

	// save the key-pair value here 
	// Note that x and y are changed during calculation, we have to use xx and yy to store the timeOptimize
	timeOptimize[ make_pair(xx, yy) ] = float(ans)/1000;
	
	return float(ans)/1000;
}

void loadIndex()
{
	long long n;
	inBufL degBuf(degName);
	inBufS inLabel(inName), outLabel(outName);
	
	n = checkB(degName)/sizeof(edgeL);

	deg = (edgeL *)malloc(sizeof(edgeL)*n);
	labelin = (edgeS*)malloc(checkB(inName));
	labelout = (edgeS*)malloc(checkB(outName));

	printf("%lld vertices\n", n);

	degBuf.start();
	for (int i = 0; i < n; i++)
		degBuf.nextEdge(deg[i]);

	inLabel.start();
	for (int i = 0; !inLabel.isEnd; i++)
		inLabel.nextEdge(labelin[i]);
	
	outLabel.start();
	for (int i = 0; !outLabel.isEnd; i++)
		outLabel.nextEdge(labelout[i]);			
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

vector< unordered_map<int, vector<request> > > loadDataset(char * datasetName, string location) {
	
	vector< unordered_map<int, vector<request> > > sourceTimeDestination; 
	
	ifstream file;
	file.open( datasetName );
	string s;
	sourceTimeDestination.resize( nodeID.size() );
	//passengerRequest.resize( MAX_END_TIME / DELTA_TIME );
	int trips = 0;

	int startTime  = 0;
	int endTime = 24*60;
	int DAY = 24;

	set<int> weekdays;
	int NY_WEEKDAYS[] = {2,3,4,7,8,9,10,11,14,15,16,17,18,22,23,24,25,28,29,30,31};
	weekdays.insert(NY_WEEKDAYS, NY_WEEKDAYS + sizeof(NY_WEEKDAYS) / sizeof(int));

	while( getline(file, s) ) 
	{
		stringstream ss( s );
		int date, source, timeSlot, dest;
		double rev = 0.0;
		ss>>date>>source>>timeSlot>>dest;
		if (location.compare("NY") == 0 || location.compare("SG") == 0) {
			ss >> rev;
		}
		

		timeSlot /= DELTA_TIME;
		// printf("Hello %d %d %d %d\n",date,source,timeSlot,dest);
		if (date > DAY) continue;
		if (weekdays.find(date) == weekdays.end()) continue;
		if (timeSlot >= endTime / DELTA_TIME) continue;

		if (source < nodeID.size() && dest < nodeID.size() && timeSlot >= startTime / DELTA_TIME) {
			trips++;

			request req;
			req.id = trips; req.source = source; req.timeSlot = timeSlot; req.destination = dest; req.revenue = rev;
			// printf("trips %d: D%d T%d %d(%.4f,%.4f)->%d(%.4f,%.4f)\n",trips,date,timeSlot,source,nodesToLatLon[source].first,nodesToLatLon[source].second,dest,nodesToLatLon[dest].first,nodesToLatLon[dest].second);

			if( date < DAY ) {
				sourceTimeDestination[ source ][ timeSlot/MINUTE_PER_HISTORY_SLOT ].push_back( req );
			}
			//else {
			//	passengerRequest[ timeSlot ][ source ].push_back( req );	
			//}
		}
		
	}

	file.close();
	//cout<<trips<<endl;
	return sourceTimeDestination;
}