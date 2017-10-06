/*
 * mapping.cpp
 *
 *  Created on: Jul 2, 2017
 *      Author: maximus
 */

#include <iostream>
#include <string>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include <std_msgs/UInt8.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/Empty.h>

#include <mobility/FindTarget.h>
#include <mobility/Obstacle.h>

using namespace std;

string rover;

grid_map::GridMap obstacle_map;
grid_map::GridMap target_map;

ros::Publisher obstaclePublish;
ros::Publisher obstacle_map_publisher;
ros::Publisher target_map_publisher;
geometry_msgs::Pose2D currentLocation;
tf::TransformListener *cameraTF;

double collisionDistance = 0.6; //meters the ultrasonic detectors will flag obstacles
unsigned int obstacle_status;

void sonarHandler(const sensor_msgs::Range::ConstPtr& sonarLeft, const sensor_msgs::Range::ConstPtr& sonarCenter, const sensor_msgs::Range::ConstPtr& sonarRight) {

	static unsigned int prev_status = 0;
	unsigned int next_status = 0;

	// TODO: Implement filtering for sonar.

	if (sonarLeft->range < collisionDistance) {
		next_status |= mobility::Obstacle::SONAR_LEFT;
	}
	if (sonarRight->range < collisionDistance) {
		next_status |= mobility::Obstacle::SONAR_RIGHT;
	}
	if (sonarCenter->range < collisionDistance) {
		next_status |= mobility::Obstacle::SONAR_CENTER;
	}
	if (sonarCenter->range < 0.12) {
		//block in front of center ultrasound.
		next_status |= mobility::Obstacle::SONAR_BLOCK;
	}

	grid_map::Polygon view_poly;
	view_poly.setFrameId(obstacle_map.getFrameId());

	view_poly.addVertex(grid_map::Position(currentLocation.x, currentLocation.y));

	// Left sonar
	view_poly.addVertex(
			grid_map::Position(currentLocation.x + 3 * cos(currentLocation.theta + M_PI_4),
					currentLocation.y + 3 * sin(currentLocation.theta + M_PI_4)
			));

	// Center sonar
	view_poly.addVertex(
			grid_map::Position(currentLocation.x + 3 * cos(currentLocation.theta),
					currentLocation.y + 3 * sin(currentLocation.theta)
			));

	// Right sonar
	view_poly.addVertex(
			grid_map::Position(currentLocation.x + 3 * cos(currentLocation.theta - M_PI_4),
					currentLocation.y + 3 * sin(currentLocation.theta - M_PI_4)
			));

	// Increase the "obstacleness" of the viewable area.
	for (grid_map::PolygonIterator iterator(obstacle_map, view_poly);
	      !iterator.isPastEnd(); ++iterator) {
		double val = obstacle_map.at("obstacles", *iterator);
		if (isnan(val)) {
			val = 0.5;
		}
		val += 0.01;
		if (val > 1)
			val = 1;
	    obstacle_map.at("obstacles", *iterator) = val;
	  }


	grid_map::Polygon clear_poly;
	clear_poly.setFrameId(obstacle_map.getFrameId());

	clear_poly.addVertex(grid_map::Position(currentLocation.x, currentLocation.y));

	// Left sonar
	clear_poly.addVertex(
			grid_map::Position(currentLocation.x + sonarLeft->range * cos(currentLocation.theta + M_PI_4),
					currentLocation.y + sonarLeft->range * sin(currentLocation.theta + M_PI_4)
			));

	// Center sonar
	clear_poly.addVertex(
			grid_map::Position(currentLocation.x + sonarCenter->range * cos(currentLocation.theta),
					currentLocation.y + sonarCenter->range * sin(currentLocation.theta)
			));

	// Right sonar
	clear_poly.addVertex(
			grid_map::Position(currentLocation.x + sonarRight->range * cos(currentLocation.theta - M_PI_4),
					currentLocation.y + sonarRight->range * sin(currentLocation.theta - M_PI_4)
			));

	// Clear the area that's clear in sonar.
	for (grid_map::PolygonIterator iterator(obstacle_map, clear_poly);
	      !iterator.isPastEnd(); ++iterator) {
		double val = obstacle_map.at("obstacles", *iterator);
		if (!isnan(val)) {
			val -= 0.05;
			if (val < 0)
				val = 0;
		}
	    obstacle_map.at("obstacles", *iterator) = val;
	  }

	if (next_status != prev_status) {
		mobility::Obstacle msg;
		msg.msg = next_status;
		msg.mask = mobility::Obstacle::IS_SONAR;
		obstaclePublish.publish(msg);
	}

	prev_status = next_status;
}

void targetHandler(const apriltags_ros::AprilTagDetectionArray::ConstPtr& message) {
	static unsigned int prev_status = 0;
	unsigned int next_status = 0;

	if (message->detections.size() > 0) {
		try {
			cameraTF->waitForTransform(rover + "/odom", ros::Time::now(),
					message->detections[0].pose.header.frame_id,
					message->detections[0].pose.header.stamp, rover + "/map",
					ros::Duration(0.25));
			for (int i=0; i<message->detections.size(); i++) {

				if (message->detections[i].id == 0)
					next_status |= mobility::Obstacle::TAG_TARGET;
				else if (message->detections[i].id == 256)
					next_status |= mobility::Obstacle::TAG_HOME;

				geometry_msgs::PoseStamped tagpose;
				cameraTF->transformPose(rover + "/odom",
						message->detections[i].pose, tagpose);
				grid_map::Position pos (tagpose.pose.position.x, tagpose.pose.position.y);
				grid_map::Index ind;
				target_map.getIndex(pos, ind);
				target_map.at("targets", ind) = 1;
			}
		} catch (tf::TransformException &e) {
			ROS_ERROR("%s", e.what());
			return;
		}
	}

	if (next_status != prev_status) {
		mobility::Obstacle msg;
		msg.msg = next_status;
		msg.mask = mobility::Obstacle::IS_VISION;
		obstaclePublish.publish(msg);
	}

	prev_status = next_status;
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr& message) {
    double x = message->pose.pose.position.x;
    double y = message->pose.pose.position.y;

    grid_map::Position newPos(x, y);
    target_map.move(newPos);

    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    currentLocation.x = x;
    currentLocation.y = y;
    currentLocation.theta = yaw;
}

bool find_neareset_target(mobility::FindTarget::Request &req, mobility::FindTarget::Response &resp) {
	bool rval = false;
	grid_map::Position mypos(currentLocation.x, currentLocation.y);
	for (grid_map::SpiralIterator it(target_map, mypos, 2); !it.isPastEnd(); ++it) {
		if (target_map.at("targets", *it) == 0) {
			// Found one!
			resp.result.header.frame_id = target_map.getFrameId();
			resp.result.header.stamp = ros::Time::now();
			grid_map::Position tagpos;
			obstacle_map.getPosition(*it, tagpos);
			resp.result.point.x = tagpos.x();
			resp.result.point.y = tagpos.y();
			resp.result.point.z = 0; // No z() in the map.
			rval = true;
			break;
		}
	}
	return rval;
}

bool clear_target_map(std_srvs::Empty::Request &req, std_srvs::Empty::Response &rsp) {
	target_map.clear("targets");
	return true;
}

void publishMap(const ros::TimerEvent& event) {
	ros::Time time = ros::Time::now();
	grid_map_msgs::GridMap message;

	//
	// Aging. As time goes by, the probability that a block
	// Is still in the place we saw it shrinks. Let's iterate
	// over the map and apply aging.
	//
	grid_map::Matrix &data = target_map["targets"];
	for (grid_map::GridMapIterator it(target_map); !it.isPastEnd(); ++it) {
		const int i = it.getLinearIndex();
		data(i) = data(i) * 0.99;
	}

	if (target_map_publisher.getNumSubscribers() > 0) {
		target_map.setTimestamp(time.toNSec());
		grid_map::GridMapRosConverter::toMessage(target_map, message);
		target_map_publisher.publish(message);
	}

	if (obstacle_map_publisher.getNumSubscribers() > 0) {
		obstacle_map.setTimestamp(time.toNSec());
		grid_map::GridMapRosConverter::toMessage(obstacle_map, message);
		obstacle_map_publisher.publish(message);
	}
}

int main(int argc, char **argv) {

	char host[128];
    gethostname(host, sizeof (host));
    string hostname(host);

    if (argc >= 2) {
        rover = argv[1];
        cout << "Welcome to the world of tomorrow " << rover
             << "!  Mapping node started." << endl;
    } else {
        rover = hostname;
        cout << "No Name Selected. Default is: " << rover << endl;
    }

    ros::init(argc, argv, (rover + "_MAP"));
    ros::NodeHandle mNH;

    obstacle_status = mobility::Obstacle::PATH_IS_CLEAR;

    // Subscribers

    ros::Subscriber odomSubscriber = mNH.subscribe((rover + "/odom/filtered"), 10, odometryHandler);
    ros::Subscriber targetSubscriber = mNH.subscribe((rover + "/targets"), 10, targetHandler);

    message_filters::Subscriber<sensor_msgs::Range> sonarLeftSubscriber(mNH, (rover + "/sonarLeft"), 10);
    message_filters::Subscriber<sensor_msgs::Range> sonarCenterSubscriber(mNH, (rover + "/sonarCenter"), 10);
    message_filters::Subscriber<sensor_msgs::Range> sonarRightSubscriber(mNH, (rover + "/sonarRight"), 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> sonarSyncPolicy;

    message_filters::Synchronizer<sonarSyncPolicy> sonarSync(sonarSyncPolicy(10), sonarLeftSubscriber, sonarCenterSubscriber, sonarRightSubscriber);
    sonarSync.registerCallback(boost::bind(&sonarHandler, _1, _2, _3));

    // Transform Listener

    cameraTF = new tf::TransformListener(ros::Duration(10));

    //	Publishers

    obstaclePublish = mNH.advertise<mobility::Obstacle>((rover + "/obstacle"), 1, true);

    obstacle_map_publisher = mNH.advertise<grid_map_msgs::GridMap>(rover + "/obstacle_map", 1, false);
    target_map_publisher = mNH.advertise<grid_map_msgs::GridMap>(rover + "/target_map", 1, false);

    // Services

    ros::ServiceServer fnt = mNH.advertiseService(rover + "/map/find_nearest_target", find_neareset_target);
    ros::ServiceServer ctm = mNH.advertiseService(rover + "/map/clear_target_map", clear_target_map);

    // Initialize the maps.
    obstacle_map = grid_map::GridMap({"obstacles"});
    obstacle_map.setFrameId(rover + "/odom");
    obstacle_map.setGeometry(grid_map::Length(25, 25), 0.5);

    target_map = grid_map::GridMap({"targets"});
    target_map.setFrameId(rover + "/odom");
    target_map.setGeometry(grid_map::Length(3, 3), 0.03);

    // Publish map updates.
    ros::Timer map_update = mNH.createTimer(ros::Duration(1), publishMap);

    ros::spin();
    return 0;
}
