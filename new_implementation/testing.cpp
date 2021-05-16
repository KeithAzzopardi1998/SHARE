#include "demand_prediction.h"
#include "ioD.h"
#include "routing.h"
#include "utils.h"

int main(int argc, char const *argv	[])
{
    pair<double, vector<int> > path;

	takeGraphInput("../ny_location","../ny_edge_time","NY");


    int source = 49421;
    int destination = 19889;

    vector<int> destinations(1);
    destinations[0] = destination;

    int timeSlot = 480;

    double alpha = 1.2;

    vector<double> actualTravelDists(destinations.size());
    actualTravelDists[0] = 0.0;

    vector<double> shortestPathDistances(destinations.size());
    shortestPathDistances[0] = query(source,destination);
    
    double maxDepth = 0.2;
    
    vector< unordered_map<int, vector<request> > > dataset = loadDataset("../ny_output_price","NY");

	vector< double > distanceFromSource( nodeID.size() );
	vector<vector< double > > distanceFromDestination( destinations.size() );
	vector<vector< double > > distanceToDestination( destinations.size() );

    int firstDestination = destinations[0];
    if(distanceFromSource[firstDestination] == MAX_DISTANCE || !distanceFromSource[ firstDestination ] ) {
		printf("Cannot reach destination?\n");
		vector<int> emptyPath;
		exit;
	}

	for (int i=0; i<destinations.size(); i++) {
		distanceToDestination[i].resize( nodeID.size() );
		distanceFromDestination[i].resize( nodeID.size() );
		dijkstra_lengths(nodeID.size(), destinations[i], source, distanceToDestination[i], edgesReverse, edgeWeightReverse);
		dijkstra_lengths(nodeID.size(), destinations[i], source, distanceFromDestination[i], edges, edgeWeight);
	}

    path = findDAGExtendedPath( nodeID.size(),
                                source,
                                destinations,
                                timeSlot/MINUTE_PER_HISTORY_SLOT,
                                alpha,
                                actualTravelDists,
                                shortestPathDistances,
                                maxDepth,
                                dataset,
                                distanceFromSource,
                                distanceToDestination,
                                distanceFromDestination,
                                edges,
                                edgeWeight);
}