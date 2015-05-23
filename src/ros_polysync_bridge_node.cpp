#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>

// ROS Includes
#include "ros/ros.h"
//#include "sensor_msgs/PointCloud.h"
//#include "sensor_msgs/LaserScan.h"
//#include "tf/transform_broadcaster.h"
#include "std_msgs/String.h"
#include "ros_polysync_bridge/IbeoObject2.h"
#include "ros_polysync_bridge/IbeoObjectArray.h"



// PolySync Includes
#include "polysync_core.h"


// *****************************************************
// global data
// *****************************************************

ros::Publisher ibeo_object_pub;
int ret;


// *****************************************************
// static declarations
// *****************************************************



/**
 * @brief API on_data handler for object stream messages.
 *
 * @warning Do not modify topic_data.
 */
static void on_psync_data_ps_object_stream_msg( void *usr_data, ps_msg_type msg_type, void *topic_data );


// *****************************************************
// static definitions
// *****************************************************


static void on_psync_data_ps_object_stream_msg( void *usr_data, ps_msg_type msg_type, void *topic_data )
{
	// cast reference
	ps_object_stream_msg *message = (ps_object_stream_msg*) topic_data;
	char buffer [50];

    psync_log_message( LOG_LEVEL_INFO, "ps_object_stream_msg -- timestamp: %llu", message->header.timestamp );
	ros_polysync_bridge::IbeoObjectArray msg_IbeoObjectArray;
	msg_IbeoObjectArray.header.stamp =ros::Time().fromNSec(message->header.timestamp);
	sprintf (buffer, "%lu_%du", message->sensor_descriptor.serial_number,message->sensor_descriptor.type);

	msg_IbeoObjectArray.header.frame_id =buffer;
	
	ROS_INFO("ps_object_stream_msg -- timestamp: %llu # of objects: %d",   message->header.timestamp, message->objects._length);
	
	for( int idx=0; idx< message->objects._length; idx++ )
	{
		ros_polysync_bridge::IbeoObject2 msg_IbeoObject2;
		msg_IbeoObject2.header.stamp 				= ros::Time().fromNSec(message->objects._buffer[ idx ].timestamp);
		msg_IbeoObject2.m_objectId 					= message->objects._buffer[ idx ].id;
		//msg_IbeoObject2.m_flags					
		msg_IbeoObject2.m_objectAge					= message->objects._buffer[ idx ].age;
		msg_IbeoObject2.m_hiddenStatusAge			= message->objects._buffer[ idx ].prediction_age;
		msg_IbeoObject2.m_classification 			= message->objects._buffer[ idx ].classification_type;
		msg_IbeoObject2.m_classificationAge			= message->objects._buffer[ idx ].classification_age;
		msg_IbeoObject2.m_classificationQuality		= message->objects._buffer[ idx ].classification_certainty;
		msg_IbeoObject2.m_centerPoint_x				= message->objects._buffer[ idx ].pos[0];
		msg_IbeoObject2.m_centerPoint_y				= message->objects._buffer[ idx ].pos[1];
		msg_IbeoObject2.m_centerPointSigma_x		= message->objects._buffer[ idx ].pos_sigma[0];
		msg_IbeoObject2.m_centerPointSigma_y		= message->objects._buffer[ idx ].pos_sigma[1];
		msg_IbeoObject2.m_courseAngle				= message->objects._buffer[ idx ].orientation;
		//msg_IbeoObject2.m_courseAngleSigma				
		msg_IbeoObject2.m_relativeVelocity_x				= message->objects._buffer[ idx ].vel[0];
		msg_IbeoObject2.m_relativeVelocity_y				= message->objects._buffer[ idx ].vel[1];
		msg_IbeoObject2.m_relativeVelocitySigma_x				= message->objects._buffer[ idx ].vel_sigma[0];
		msg_IbeoObject2.m_relativeVelocitySigma_y				= message->objects._buffer[ idx ].vel_sigma[1];
		//msg_IbeoObject2.m_absoluteVelocity_x				
		//msg_IbeoObject2.m_absoluteVelocity_y				
		//msg_IbeoObject2.m_absoluteVelocitySigma_x				
		//msg_IbeoObject2.m_absoluteVelocitySigma_y				
		msg_IbeoObject2.m_objectBox_x				= message->objects._buffer[ idx ].size[0];
		msg_IbeoObject2.m_objectBox_y				= message->objects._buffer[ idx ].size[1];
		//msg_IbeoObject2.m_objectBoxSigma_x		
		//msg_IbeoObject2.m_objectBoxSigma_y		
		//msg_IbeoObject2.m_boundingBoxCenter_x		
		//msg_IbeoObject2.m_boundingBoxCenter_y	
		//msg_IbeoObject2.m_boundingBox_x				
		//msg_IbeoObject2.m_boundingBox_y				
		//msg_IbeoObject2.m_closestPoint_x				
		//msg_IbeoObject2.m_closestPoint_y				
		//msg_IbeoObject2.m_contourPoints_x.push_back();
		//msg_IbeoObject2.m_contourPoints_y.push_back();
		//msg_IbeoObject2.m_vehicleWLANid
		//msg_IbeoObject2.m_objectHeight
		//msg_IbeoObject2.m_objectHeightSigma
		//msg_IbeoObject2.m_objectMass
		//msg_IbeoObject2.m_isValid

		
		msg_IbeoObjectArray.objects.push_back(msg_IbeoObject2);
		
	}
	ibeo_object_pub.publish(msg_IbeoObjectArray);
/*
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world ";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());


    chatter_pub.publish(msg);*/
}

int polysync_GRACEFUL_EXIT_STMNT()
{
	if( (ret = psync_release( 0 )) != DTC_RET( DTC_NONE ) )
		{
			psync_log_message( LOG_LEVEL_ERROR, "main -- psync_release - ret: %d", ret );
		}
		return -1;
}
// *****************************************************
// main
// *****************************************************
int main( int argc, char **argv )
{
    // polysync return status
    ret = DTC_RET( DTC_NONE );

    // polysync node name
    const char *node_name = "ROS_PolySynce_Bridge";
    ros::init(argc, argv, "ROS_PolySynce_Bridge");
	ros::NodeHandle n;
	ibeo_object_pub = n.advertise<ros_polysync_bridge::IbeoObjectArray>("polysync_ibeo_object_stream", 1000);
	//ros::Rate loop_rate(10);
    // flag to enable stdout logs in addition to the normal syslog output
    unsigned int stdout_logging_enabled = 1;



	// init core API
    if( (ret = psync_init( PSYNC_NID_API, node_name, stdout_logging_enabled )) != DTC_RET( DTC_NONE ) )
    {
        psync_log_message( LOG_LEVEL_ERROR, "main -- psync_init - ret: %d", ret );
		return polysync_GRACEFUL_EXIT_STMNT();
    }

    //
    // set event message subscribers to have RELIABLE QoS
    if( (ret = psync_set_subscriber_reliability_qos( MSG_TYPE_EVENT, RELIABILITY_QOS_RELIABLE )) != DTC_RET( DTC_NONE ) )
    {
        psync_log_message( LOG_LEVEL_ERROR, "main -- psync_set_subscriber_reliability_qos - ret: %d", ret );
        return polysync_GRACEFUL_EXIT_STMNT();
    }




    // register a listener for object stream messages
    if( (ret = psync_message_register_listener( MSG_TYPE_OBJECT_STREAM, on_psync_data_ps_object_stream_msg, NULL )) != DTC_RET( DTC_NONE ) )
    {
        psync_log_message( LOG_LEVEL_ERROR, "main -- psync_message_register_listener - ret: %d", ret );
        return polysync_GRACEFUL_EXIT_STMNT();
    }
/*
    // register a listener for lane model messages
    if( (ret = psync_message_register_listener( MSG_TYPE_LANE_MODEL, on_psync_data_ps_lane_model_msg, NULL )) != DTC_RET( DTC_NONE ) )
    {
        psync_log_message( LOG_LEVEL_ERROR, "main -- psync_message_register_listener - ret: %d", ret );
        return polysync_GRACEFUL_EXIT_STMNT();
    }

    // register a listener for platform motion messages
    if( (ret = psync_message_register_listener( MSG_TYPE_PLATFORM_MOTION, on_psync_data_ps_platform_motion_msg, NULL )) != DTC_RET( DTC_NONE ) )
    {
        psync_log_message( LOG_LEVEL_ERROR, "main -- psync_message_register_listener - ret: %d", ret );
        return polysync_GRACEFUL_EXIT_STMNT();
    }

    // register a listener for event messages
    if( (ret = psync_message_register_listener( MSG_TYPE_EVENT, on_psync_data_ps_event_msg, NULL )) != DTC_RET( DTC_NONE ) )
    {
        psync_log_message( LOG_LEVEL_ERROR, "main -- psync_message_register_listener - ret: %d", ret );
        return polysync_GRACEFUL_EXIT_STMNT();
    }

    // register a listener for parameter stream messages
    if( (ret = psync_message_register_listener( MSG_TYPE_PARAMETER_STREAM, on_psync_data_ps_parameter_stream_msg, NULL )) != DTC_RET( DTC_NONE ) )
    {
        psync_log_message( LOG_LEVEL_ERROR, "main -- psync_message_register_listener - ret: %d", ret );
        return polysync_GRACEFUL_EXIT_STMNT();
    }

*/
	// main processing loop
	
	ros::spin();
	polysync_GRACEFUL_EXIT_STMNT();
	return 0;
}




//#include <node.h> //in c++ api
//#include <message.h> //in c++ api
/*
    void gotTracks( const polysync::message::RadarTrackStream& msg );
    void gotPoints( const polysync::message::LiDARPointStream& msg );
    
  
void OccupancyGrid::handleMessage( void* msg )
{
    ps_msg_type type = polysync::message::getType( msg );

    if( type == MSG_TYPE_RADAR_TRACK_STREAM )
    {
        gotTracks( polysync::message::RadarTrackStream( msg ) );
    }
    else if( type == MSG_TYPE_LIDAR_POINT_STREAM )
    {
        gotPoints( polysync::message::LiDARPointStream( msg ) );
    }
}

void OccupancyGrid::gotTracks( const polysync::message::RadarTrackStream& msg )
{
    _gotTracks = true;
    std::vector< ps_radar_track > vector = msg.dataStreamVector();

    for( auto track : vector )
    {
        updateGrid( track.pos );
    }
}

void OccupancyGrid::gotPoints( const polysync::message::LiDARPointStream& msg )
{
    _gotPoints = true;
    std::vector< ps_lidar_point > vector = msg.dataStreamVector();

    for( auto point : vector )
    {
        updateGrid( point.pos );
    }
}track.serialNumber()  
int main()
{
    polysync::Node p_node( "ROS PolySynce Bridge" );
    node.registerListener( MSG_TYPE_LIDAR_POINT_STREAM );
	registerListener( MSG_TYPE_RADAR_TRACK_STREAM );
    registerListener( MSG_TYPE_LIDAR_POINT_STREAM );
    
    // register a listener for lane model messages
    if( (ret = psync_message_register_listener( MSG_TYPE_LANE_MODEL, on_psync_data_ps_lane_model_msg, NULL )) != DTC_RET( DTC_NONE ) )
    {
        psync_log_message( LOG_LEVEL_ERROR, "main -- psync_message_register_listener - ret: %d", ret );
        return polysync_GRACEFUL_EXIT_STMNT();
    }
    
    while( ! node.exitSignal() )
    {
        polysync::message::LiDARPointStream message( node.queuePopSafe() );
void* msg = queuePopSafe( true );

        if( msg != nullptr )
        {
            handleMessage( msg );
            usleep( 1000 );
        }
        if( message.hasData() )
        {
            message.print();
            message.printDataStream( 2 );
        }
    } 

    return 0;
}*/
