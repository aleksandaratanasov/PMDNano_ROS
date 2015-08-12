#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

/*
 * NO NEED TO USE THIS! Rqt already has an integrated image viewer *facepalm*
 * See Plugins -> Visualization -> Image View
 *
 */

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("Depth", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "pmd_nano_image_subscriber");
  ros::NodeHandle nh;

  //  ROS_INFO("Creating windows for displaying OpenCV output");
  //  cv::namedWindow("Depth", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
  //  cv::namedWindow("Amplitude", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);

  ROS_INFO("Subscribing to pmd_nano/depth");
  cv::namedWindow("Depth", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);
  cv::startWindowThread();

  if(!nh.ok()) {
    ROS_ERROR("NodeHandle error occurred");
    return 1;
  }

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber subDepth = it.subscribe("pmd_nano/depth", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("Depth");

  return 0;
}
