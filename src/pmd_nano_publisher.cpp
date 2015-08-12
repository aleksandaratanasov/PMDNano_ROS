#include <ros/ros.h>
#include <string>
#include <sstream>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_ros/point_cloud.h>
//#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_types.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "../include/pmd_nano/PMDNano.h"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/publisher.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/publisher.h>

/*
 * FIXME:
 *  - published images of depth and amplitude look like crap; implement a version of ROS Image that can handle CV_32F
 * TODO:
 *  - add publishing of the cloud produced by the PMD device
 *  - (optimization) see if PMDs output image can be directly converted into ROS image instead of going through the OpenCV step or at least if the data block (pixel information)
 * can be just passed along the way from one format to another
 */

using std::string;

class PMDPublisher
{
protected:
  string ppp_file, pap_file;
  ros::NodeHandle nh;
  std::shared_ptr<DepthCamera> camera;
  string cloud_topic;
  string imgDepth_topic, imgAmplitude_topic;
  string tf_frame;
  ros::NodeHandle private_nh;
  pcl_ros::Publisher<sensor_msgs::PointCloud2> pubCloud;
  image_transport::Publisher pubDepth, pubAmplitude;
public:
  sensor_msgs::PointCloud2 cloud;
  //sensor_msgs::PointCloud2Ptr cloud;

  PMDPublisher(string _ppp_file, string _pap_file, string _cloud_topic="pmd_cloud", string _imgDepth_topic="pmd_depth", string _imgAmplitude_topic="pmd_amplitude")
    : ppp_file(_ppp_file), pap_file(_pap_file),
      cloud_topic(_cloud_topic),
      imgDepth_topic(_imgDepth_topic), imgAmplitude_topic(_imgAmplitude_topic),
      tf_frame("/base_link"),
      private_nh("~")
  {
    ROS_INFO("Initializing PMD Publisher");
    if(ppp_file == "" || pap_file == "")
      exit(EXIT_FAILURE);

    ROS_INFO_STREAM("Following PMD configuration files were loaded:\n"
                    << "PPP file: \"" << ppp_file << "\"\n"
                    << "PAP file: \"" << pap_file << "\"\n");

    camera = std::shared_ptr<DepthCamera>(new PMDNano(pap_file, ppp_file));

    pubCloud.advertise(nh, cloud_topic, 1);
    image_transport::ImageTransport imgTransportDepth(nh), imgTransportAmplitude(nh);
    pubDepth = imgTransportDepth.advertise(imgDepth_topic, 1);
    pubAmplitude = imgTransportAmplitude.advertise(imgAmplitude_topic, 1);

    private_nh.param("frame_id", tf_frame, string("/base_link"));
    ROS_INFO_STREAM("Publishing data on topic \"" << nh.resolveName(cloud_topic) << "\" with frame_id \"" << tf_frame << "\"");
  }

  int start()
  {
    try {
     camera->start();
    }
    catch(std::exception e) {
      ROS_ERROR("Node unable to start camera");
      return -1;
    }
    ROS_INFO_STREAM("Depth: " << camera->depthSize().width << " by " << camera->depthSize().height);
    return 0;
  }

  bool spin ()
  {
    ROS_INFO("Allocating memory");
    cv::Mat depth = cv::Mat::zeros(camera->depthSize(), CV_32F);
    cv::Mat amplitude = cv::Mat::zeros(camera->depthSize(), CV_32F);
    cv::Mat a;
    int nr_points;
    string fields_list;
    sensor_msgs::ImagePtr depthMsg, amplitudeMsg;
    PointCloud::Ptr cloudPcl(new PointCloud(camera->depthSize().width, camera->depthSize().height));

    ros::Rate r(100);

    ROS_INFO("Entering loop");
    int count = 0;
    int fileIdx = 0;
    std::ostringstream ss1, ss2;

    while(nh.ok ())
    {
      ROS_INFO("Capturing new data");
      camera->captureDepth(depth);
      camera->captureAmplitude(amplitude);
      camera->capturePointCloud(cloudPcl);
      if(count == 100) {
        ROS_INFO("*********************************Writing point cloud data to file...");
        ss1 << "/home/latadmin/catkin_ws/devel/lib/pmd_nano/pcd_data/output_ascii_" << fileIdx << ".pcd";
        ss2 << "/home/latadmin/catkin_ws/devel/lib/pmd_nano/pcd_data/output_bincompr_" << fileIdx << ".pcd";
        pcl::io::savePCDFileASCII(ss1.str(), *cloudPcl);
        pcl::io::savePCDFileBinaryCompressed(ss2.str(), *cloudPcl);
        fileIdx++;
        ss1.str("");
        ss2.str("");
        count = 0;
      }
      else {
        ROS_INFO_STREAM("counter: " << count);
        count++;
      }
      pcl::toROSMsg(*cloudPcl, cloud);
      cloud.header.frame_id = tf_frame;
      nr_points = cloud.width * cloud.height;
      fields_list = pcl::getFieldsList(cloud);

      ROS_INFO("Flipping retrieved data and converting colour space (amplitude only!)");
      cv::flip(depth, depth, 0);
      cv::flip(amplitude, amplitude, 0);
      amplitude.convertTo(a, CV_8U, 255.0 / 1000.0);

      depthMsg = cv_bridge::CvImage(std_msgs::Header(), "mono16", depth).toImageMsg();
      amplitudeMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", amplitude).toImageMsg();

      //ROS_DEBUG_STREAM_ONCE
      ROS_INFO_STREAM("Publishing data with " << nr_points
                            << " points " << fields_list
                            << " on topic \"" << nh.resolveName(cloud_topic)
                            << "\" in frame \"" << cloud.header.frame_id << "\"");

      cloud.header.stamp = ros::Time::now();
      pubCloud.publish(cloud);
      pubDepth.publish(depthMsg);
      pubAmplitude.publish(amplitudeMsg);

      r.sleep();
    }
    return (true);
  }
};

int main(int argc, char* argv[])
{
  ros::init (argc, argv, "pmd_nano_publisher");
  ros::NodeHandle nh;
  string ppp_file = "", pap_file = "";
  ROS_INFO("Retrieveing parameters");
  nh.getParam("ppp", ppp_file);
  nh.getParam("pap", pap_file);

  if(argc == 3 && (ppp_file == "" || pap_file == "")) {
    ROS_INFO("Overloading launch parameters with command line arguments");
    ppp_file = argv[1];
    pap_file = argv[2];
  }

  PMDPublisher pmdp(ppp_file, pap_file);

  if (pmdp.start () == -1)
  {
    //ROS_ERROR_STREAM("Could not load file \"" << c.file_name  <<"\". Exiting...");
    ROS_ERROR("Dang it!");
    return -1;
  }
  /*ROS_INFO_STREAM("Loaded a point cloud with " << pmdp.cloud.width * pmdp.cloud.height
                  << " points (total size is " << pmdp.cloud.data.size() << ") and the following channels: " << pcl::getFieldsList(pmdp.cloud));*/
  pmdp.spin();

  return 0;
}

/*
using std::string;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "pmd_nano_publisher");
  ros::NodeHandle nh;

// ppp_file = "camboardnanoproc.L64.ppp", pap_file = "camboardnano.L64.pap";
// for easier use call pmd_nano_default.launch (the ppp and pap files are in the launch directory)
  string ppp_file = "", pap_file = "";
  nh.getParam("ppp", ppp_file);
  nh.getParam("pap", pap_file);

  ROS_INFO_STREAM("Loaded ppp file: " << ppp_file);
  ROS_INFO_STREAM("Loaded pap file: " << pap_file);

  ROS_INFO("Connecting to camera");
  std::shared_ptr<DepthCamera> camera(new PMDNano(pap_file, ppp_file));
  camera->start();

  cv::Mat depth = cv::Mat::zeros(camera->depthSize(), CV_32F);
  cv::Mat amplitude = cv::Mat::zeros(camera->depthSize(), CV_32F);
//  std::shared_ptr<pcl::visualization::CloudViewer> viewer(new pcl::visualization::CloudViewer("Vertex"));
  PointCloud::Ptr cloud(new PointCloud(camera->depthSize().width, camera->depthSize().height));

  ROS_INFO_STREAM("DEPTH: " << camera->depthSize().width << " by " << camera->depthSize().height);

  // Creating publishers for depth and amplitude images
  image_transport::ImageTransport imgTransportDepth(nh), imgTransportAmplitude(nh);
  image_transport::Publisher pubDepth = imgTransportDepth.advertise("pmd_nano/depth", 1);
  image_transport::Publisher pubAmplitude = imgTransportAmplitude.advertise("pmd_nano/amplitude", 1);

  // Creating a publisher for the point cloud
  ros::Publisher pubCloud = nh.advertise<PointCloud> ("pmd_nano/cloud", 10);

  double rate = 40;
  ROS_INFO_STREAM("Setting rate to " << rate);
  ros::Rate r(rate);

  while (nh.ok()) {
      ROS_INFO("Capturing new data");
      camera->captureDepth(depth);
      camera->captureAmplitude(amplitude);
      camera->capturePointCloud(cloud);

      ROS_INFO("Flipping retrieved data");
      cv::flip(depth, depth, 0);
      cv::flip(amplitude, amplitude, 0);

      cv::Mat a;
      amplitude.convertTo(a, CV_8U, 255.0 / 1000.0);
      // Publish depth and aplitude images
      // Major problem here - we have CV_32F and there is no such encoding available in cv_bridge (goes up to 16bit data structures)
      sensor_msgs::ImagePtr depthMsg = cv_bridge::CvImage(std_msgs::Header(), "mono16", depth).toImageMsg();
      sensor_msgs::ImagePtr amplitudeMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", amplitude).toImageMsg();
//      sensor_msgs::PointCloud2 cloudMsg;

      pubDepth.publish(depthMsg);
      pubAmplitude.publish(amplitudeMsg);
      //cloud->header.stamp = ros::Time::now().toNSec();
      //ROS_INFO_STREAM("Cloud publish time stamp: " << cloud->header.stamp);

      pubCloud.publish(cloud);


//      ROS_INFO("Displaying data");
      // Just to see the way OpenCV displays the image before it's converted to ROS Image and published
//      cv::imshow("Depth", depth); // Hmmmmmm, this doesn't show at all....
//      cv::waitKey(10);
//      cv::imshow("Amplitude", a);
//      viewer->showCloud(cloud);

      ros::spinOnce();
      r.sleep();
  }

  return 0;
}
*/
