#include "demand_prediction.h"
#include "ioD.h"
#include "routing.h"
#include "utils.h"

int main(int argc, char const *argv	[])
{
    pair<double, vector<int> > path;
    printf("MAIN: loading graph\n");
	takeGraphInput("../ny_location","../ny_edge_time","NY");

    printf("MAIN: loading index\n");
    loadIndex("../nyIndex.deg","../nyIndex.labelin","../nyIndex.labelout");

    printf("MAIN: setting constants\n");
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
    
    printf("MAIN: loading dataset\n");
    vector< unordered_map<int, vector<request> > > dataset = loadDataset("../ny_output_price","NY");

    printf("MAIN: calculating distances\n");
	vector< double > distanceFromSource( nodeID.size() );
	vector<vector< double > > distanceFromDestination( destinations.size() );
	vector<vector< double > > distanceToDestination( destinations.size() );
    int firstDestination = destinations[0];
    //if(distanceFromSource[firstDestination] == MAX_DISTANCE || !distanceFromSource[ firstDestination ] ) {
	//	printf("Cannot reach destination?\n");
	//	vector<int> emptyPath;
	//	exit;
	//}

	for (int i=0; i<destinations.size(); i++) {
		distanceToDestination[i].resize( nodeID.size() );
		distanceFromDestination[i].resize( nodeID.size() );
		dijkstra_lengths(nodeID.size(), destinations[i], source, distanceToDestination[i], edgesReverse, edgeWeightReverse);
		dijkstra_lengths(nodeID.size(), destinations[i], source, distanceFromDestination[i], edges, edgeWeight);
	}

    printf("MAIN: assigning extended edges\n");
    assignExtendEdge(nodeID.size(), maxDepth, edges, edgeWeight);

    printf("MAIN: generating nearby nodes\n");
    generateNearbyNodes(nodeID.size(), edges, edgeWeight);

    printf("\n\n\nMAIN: shortest distance (calculated by query): %f\n",query(source,destination));

    printf("\n\n\nMAIN: calculating path with mocked node score\n");
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
                                edgeWeight,
                                0);

    printf("\n\n\nMAIN: calculating path with baseline node score\n");
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
                                edgeWeight,
                                1);

    printf("\n\n\nMAIN: calculating path with clustered node score\n");
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
                                edgeWeight,
                                2);
}