#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class Odometry_TF {
public:
  Odometry_TF() {
    global_pub_ = n_.advertise<nav_msgs::Odometry>("tf/global_pose", 1);
    global_sub_ = n_.subscribe("odometry/global_pose", 1, &Odometry_TF::callback, this);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher global_pub_;
  ros::Subscriber global_sub_;
  tf::TransformBroadcaster odom_broadcaster;

  void callback(const geometry_msgs::Pose& input) {
    geometry_msgs::TransformStamped odom_trans;
    ros::Time current_time = ros::Time::now();
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = input.position.x;
    odom_trans.transform.translation.y = input.position.x;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = input.orientation;
    odom_broadcaster.sendTransform(odom_trans);
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = input.position.x;
    odom.pose.pose.position.y = input.position.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = input.orientation;
    odom.child_frame_id = "base_link";
    global_pub_.publish(odom);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_tf");
  Odometry_TF odometry_tf;
  ros::spin();

  return 0;
}
