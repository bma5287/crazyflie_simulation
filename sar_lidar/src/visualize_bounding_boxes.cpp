#include <ros/ros.h>
#include <sar_msgs/BoundingBoxArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

ros::Publisher marker_pub;

void bboxCallback(const sar_msgs::BoundingBoxArray::ConstPtr& msg)
{
    visualization_msgs::MarkerArray marker_array;
    int id = 0;

    for (const auto& bbox : msg->boxes)
    {
        visualization_msgs::Marker marker;
        marker.header = msg->header;
        marker.ns = "bounding_boxes";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        // Compute the center and scale from min and max points
        marker.pose.position.x = (bbox.min_point.x + bbox.max_point.x) / 2.0;
        marker.pose.position.y = (bbox.min_point.y + bbox.max_point.y) / 2.0;
        marker.pose.position.z = (bbox.min_point.z + bbox.max_point.z) / 2.0;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = bbox.max_point.x - bbox.min_point.x + 0.1;
        marker.scale.y = bbox.max_point.y - bbox.min_point.y;
        marker.scale.z = bbox.max_point.z - bbox.min_point.z;

        marker.color.a = 0.5;  // Transparency
        marker.color.r = 0.0;
        marker.color.g = 0.5;
        marker.color.b = 1.0;

        marker.lifetime = ros::Duration(10.0);

        marker_array.markers.push_back(marker);
    }

    marker_pub.publish(marker_array);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualize_bounding_boxes");
    ros::NodeHandle nh;

    ros::Subscriber bbox_sub = nh.subscribe("/SAR_Internal/lidar/bounding_boxes_raw", 1, bboxCallback);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("bounding_boxes_markers", 1);

    ros::spin();

    return 0;
}
