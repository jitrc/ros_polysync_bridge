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
#include "ros_polysync_bridge/LaneModel.h"
#include "ros_polysync_bridge/LaneModelAll.h"
#include "ros_polysync_bridge/RadarTrack.h"
#include "ros_polysync_bridge/RadarTrackArray.h"
#include "ros_polysync_bridge/LidarPoint.h"
#include "ros_polysync_bridge/LidarPointArray.h"



// PolySync Includes
#include "polysync_core.h"


// *****************************************************
// global data
// *****************************************************

ros::Publisher ibeo_object_pub;
ros::Publisher lane_model_pub;
ros::Publisher lidar_points_pub;
ros::Publisher radar_track_pub;

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
static void on_psync_data_ps_lane_model_msg(void *usr_data, ps_msg_type msg_type, void *topic_data);

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

static void on_psync_data_ps_lidar_point_stream_msg( void *usr_data, ps_msg_type msg_type, void *topic_data )
{
	// cast reference
	ps_lidar_point_stream_msg *message = (ps_lidar_point_stream_msg*) topic_data;
	char buffer [50];

    psync_log_message( LOG_LEVEL_INFO, "ps_lidar_point_stream_msg -- timestamp: %llu", message->header.timestamp );
	ros_polysync_bridge::LidarPointArray msg_LidarPointArray;
	msg_LidarPointArray.header.stamp =ros::Time().fromNSec(message->header.timestamp);
	sprintf (buffer, "%lu_%du", message->sensor_descriptor.serial_number,message->sensor_descriptor.type);

	msg_LidarPointArray.header.frame_id =buffer;
	
	ROS_INFO("ps_lidar_point_stream_msg -- timestamp: %llu # of lidar_points: %d",   message->header.timestamp, message->points._length);
	
	for( int idx=0; idx< message->points._length; idx++ )
	{
		ros_polysync_bridge::LidarPoint msg_LidarPoint;
					
		msg_LidarPoint.m_layer_kind				= message->points._buffer[ idx ].layer;
		msg_LidarPoint.m_echo_kind				= message->points._buffer[ idx ].echo;
		msg_LidarPoint.m_point_kind 			= message->points._buffer[ idx ].type;
		msg_LidarPoint.m_echo_pulse_width		= message->points._buffer[ idx ].echo_pulse_width;
		msg_LidarPoint.m_position_x				= message->points._buffer[ idx ].pos[0];
		msg_LidarPoint.m_position_y				= message->points._buffer[ idx ].pos[1];
		msg_LidarPoint.m_position_z				= message->points._buffer[ idx ].pos[2];

		msg_LidarPointArray.m_lidar_points.push_back(msg_LidarPoint);
		
	}
	lidar_points_pub.publish(msg_LidarPointArray);

}

static void on_psync_data_ps_radar_track_stream_msg( void *usr_data, ps_msg_type msg_type, void *topic_data )
{
	// cast reference
	ps_radar_track_stream_msg *message = (ps_radar_track_stream_msg*) topic_data;
	char buffer [50];

    psync_log_message( LOG_LEVEL_INFO, "ps_radar_track_stream_msg -- timestamp: %llu", message->header.timestamp );
	ros_polysync_bridge::RadarTrackArray msg_RadarTrackArray;
	msg_RadarTrackArray.header.stamp =ros::Time().fromNSec(message->header.timestamp);
	sprintf (buffer, "%lu_%du", message->sensor_descriptor.serial_number,message->sensor_descriptor.type);

	msg_RadarTrackArray.header.frame_id =buffer;
	
	ROS_INFO("ps_radar_track_stream_msg -- timestamp: %llu # of radar_tracks: %d",   message->header.timestamp, message->tracks._length);
	
	for( int idx=0; idx< message->tracks._length; idx++ )
	{
		ros_polysync_bridge::RadarTrack msg_RadarTrack;
		msg_RadarTrack.header.stamp 				= ros::Time().fromNSec(message->tracks._buffer[ idx ].timestamp);
		msg_RadarTrack.m_objectId 					= message->tracks._buffer[ idx ].id;		
		msg_RadarTrack.m_track_status				= message->tracks._buffer[ idx ].status;
		msg_RadarTrack.m_position_x				= message->tracks._buffer[ idx ].pos[0];
		msg_RadarTrack.m_position_y				= message->tracks._buffer[ idx ].pos[1];
		msg_RadarTrack.m_position_z				= message->tracks._buffer[ idx ].pos[2];
		msg_RadarTrack.m_velocity_x 			= message->tracks._buffer[ idx ].vel[0];
		msg_RadarTrack.m_velocity_y 			= message->tracks._buffer[ idx ].vel[1];
		msg_RadarTrack.m_velocity_z 			= message->tracks._buffer[ idx ].vel[2];
		msg_RadarTrack.m_amplitude		        = message->tracks._buffer[ idx ].amplitude;
		msg_RadarTrack.m_range_mode				= message->tracks._buffer[ idx ].range_mode;
		msg_RadarTrack.m_scan_index				= message->tracks._buffer[ idx ].scan_index;

		msg_RadarTrackArray.m_radar_tracks.push_back(msg_RadarTrack);
		
	}
	radar_track_pub.publish(msg_RadarTrackArray);

}

static void on_psync_data_ps_lane_model_msg(void *usr_data, ps_msg_type msg_type, void *topic_data) 
{
	// cast reference
	ps_lane_model_msg *message = (ps_lane_model_msg*) topic_data;
	char buffer [50];

    psync_log_message( LOG_LEVEL_INFO, "ps_lane_model_msg -- timestamp: %llu", message->header.timestamp );
	ros_polysync_bridge::LaneModelAll msg_LaneModelAll;
	msg_LaneModelAll.header.stamp =ros::Time().fromNSec(message->header.timestamp);
	sprintf (buffer, "%lu_%du", message->sensor_descriptor.serial_number,message->sensor_descriptor.type);

	msg_LaneModelAll.header.frame_id =buffer;
	
	ROS_INFO("ps_lane_model_msg -- timestamp: %llu # of lanes: %d",   message->header.timestamp, 2 + message->additional_lanes._length);
	
	   ros_polysync_bridge::LaneModel msg_left_lane;
	   ros_polysync_bridge::LaneModel msg_right_lane;
	   ros_polysync_bridge::LaneModel msg_lane;
	   
		//msg_left_lane.m_is_valid 				    = message->left_lane.is_valid;
		//msg_IbeoObject2.m_detection_confidence 					= message->objects._buffer[ idx ].id;
		//msg_IbeoObject2.m_flags					
		msg_left_lane.m_marker_width		= message->left_lane.marker_width;
		msg_left_lane.m_heading_angle		= message->left_lane.heading_angle;
		msg_left_lane.m_view_range 			= message->left_lane.view_range;
		msg_left_lane.m_time_to_crossing    = message->left_lane.time_to_crossing;
		msg_left_lane.m_lane_crossing		= message->left_lane.lane_crossing;
		msg_left_lane.m_lane_offset			= message->left_lane.lane_offset;
		msg_left_lane.m_curvature			= message->left_lane.curvature;
		msg_left_lane.m_curvature_derivative		= message->left_lane.curvature_derivative;
		
	    msg_right_lane.m_marker_width		= message->right_lane.marker_width;
		msg_right_lane.m_heading_angle		= message->right_lane.heading_angle;
		msg_right_lane.m_view_range 		= message->right_lane.view_range;
		msg_right_lane.m_time_to_crossing   = message->right_lane.time_to_crossing;
		msg_right_lane.m_lane_crossing		= message->right_lane.lane_crossing;
		msg_right_lane.m_lane_offset		= message->right_lane.lane_offset;
		msg_right_lane.m_curvature			= message->right_lane.curvature;
		msg_right_lane.m_curvature_derivative		= message->right_lane.curvature_derivative;
		
		//msg_IbeoObjectArray.objects.push_back(msg_IbeoObject2);
		msg_LaneModelAll.m_left_lane = msg_left_lane;
		msg_LaneModelAll.m_right_lane = msg_right_lane;
		
		msg_LaneModelAll.m_reference_point_x = message->reference_point[0];
		msg_LaneModelAll.m_reference_point_y = message->reference_point[1];
		msg_LaneModelAll.m_reference_point_z = message->reference_point[2];
		
		
		for(int id = 0; id < message->additional_lanes._length; id++){
		    msg_lane.m_marker_width		= message->additional_lanes._buffer[id].marker_width;
		    msg_lane.m_heading_angle	= message->additional_lanes._buffer[id].heading_angle;
			msg_lane.m_view_range 		= message->additional_lanes._buffer[id].view_range;
			msg_lane.m_time_to_crossing = message->additional_lanes._buffer[id].time_to_crossing;
			msg_lane.m_lane_crossing	= message->additional_lanes._buffer[id].lane_crossing;
			msg_lane.m_lane_offset		= message->additional_lanes._buffer[id].lane_offset;
			msg_lane.m_curvature		= message->additional_lanes._buffer[id].curvature;
			msg_lane.m_curvature_derivative	= message->additional_lanes._buffer[id].curvature_derivative;
		    
		    msg_LaneModelAll.m_additional_lanes.push_back(msg_lane);		    
		}
		
	
	lane_model_pub.publish(msg_LaneModelAll);
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
	lane_model_pub = n.advertise<ros_polysync_bridge::LaneModelAll>("polysync_lane_model_all", 1000);
	lidar_points_pub = n.advertise<ros_polysync_bridge::LidarPointArray>("polysync_lidar_points_stream", 1000);
	radar_track_pub = n.advertise<ros_polysync_bridge::RadarTrackArray>("polysync_radar_track_stream", 1000);
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

    // register a listener for lane model messages
    if( (ret = psync_message_register_listener( MSG_TYPE_LANE_MODEL, on_psync_data_ps_lane_model_msg, NULL )) != DTC_RET( DTC_NONE ) )
    {
        psync_log_message( LOG_LEVEL_ERROR, "main -- psync_message_register_listener - ret: %d", ret );
        return polysync_GRACEFUL_EXIT_STMNT();
    }
    
   // register a listener for lidar point stream messages
        if( (ret = psync_message_register_listener( MSG_TYPE_LIDAR_POINT_STREAM , on_psync_data_ps_lidar_point_stream_msg, NULL )) != DTC_RET( DTC_NONE ) )
    {
        psync_log_message( LOG_LEVEL_ERROR, "main -- psync_message_register_listener - ret: %d", ret );
        return polysync_GRACEFUL_EXIT_STMNT();
    }
    
   // register a listener for radar track stream messages
        if( (ret = psync_message_register_listener( MSG_TYPE_RADAR_TRACK_STREAM , on_psync_data_ps_radar_track_stream_msg, NULL )) != DTC_RET( DTC_NONE ) )
    {
        psync_log_message( LOG_LEVEL_ERROR, "main -- psync_message_register_listener - ret: %d", ret );
        return polysync_GRACEFUL_EXIT_STMNT();
    }

/*
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
