#include "ros/ros.h"
#include "ObservationService/Observation.h"
#include <cstdlib>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Observation_client");
	if (argc != 3)
	{
		ROS_INFO("WRONG PARAMS");
		return 1;
	}

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<ObservationService::Observation>("Observations");
	ObservationService::Observation srv;
    srv.request.state = atoll(argv[1]);
    srv.request.action = atoll(argv[2]);

     if (client.call(srv))
     {
       ROS_INFO("Observation: %ld", (long int)srv.response.observation);
     }
     else
     {
       ROS_ERROR("Failed to call service");
       return 1;
     }
   
     return 0;
}