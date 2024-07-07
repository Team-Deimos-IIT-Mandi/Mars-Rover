#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <std_msgs/String.h>

std::vector<double> distortion_coefficients;
cv::Mat matrix_coefficients;
geometry_msgs::PoseStamped betweenPose;
geometry_msgs::PoseStamped postPose;
std::string mode = "P";

class TestNode
{
public:
  TestNode() : nh_()
  {
    test_pub_ = nh_.advertise<std_msgs::String>("test_topic", 10); // Advertise to "test_topic"

    // Publish a message every second
    timer_ = nh_.createTimer(ros::Duration(1.0), &TestNode::timerCallback, this);
  }

  void timerCallback(const ros::TimerEvent &event)
  {

    ROS_INFO("Timer callback triggered at %f", event.current_real.toSec());

    std_msgs::String msg;
    msg.data = "Testing marker_detection code";
    test_pub_.publish(msg);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher test_pub_;
  ros::Timer timer_;
};

class ImageConverter
{
public:
  ImageConverter() : it_(nh_)
  {
    image_pub_ = it_.advertise("between_image", 1);
    detected_aruco_image_pub_ = nh_.advertise<std_msgs::String>("detected_aruco_image", 1); // Publisher for std_msgs::String
    AR_pub_ = nh_.advertise<std_msgs::Bool>("AR", 1);
    between_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ImageConverter::imageCallback, this);
    camera_info_sub_ = nh_.subscribe("/camera/color/camera_info", 1, &ImageConverter::cameraInfoCallback, this);
  }

  ~ImageConverter() {}

  void imageCallback(const sensor_msgs::ImageConstPtr &msg)
  {
    try
    {
      cv::Mat cv_image = cv_bridge::toCvShare(msg, "bgr8")->image;
      cv::Mat cv_image_gray;
      cv::cvtColor(cv_image, cv_image_gray, cv::COLOR_BGR2GRAY);

      cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));

      cv::Mat cv_image_gray_clahe;
      clahe->apply(cv_image_gray, cv_image_gray_clahe);
      cv_image_gray = cv_image_gray_clahe;

      // cv_image_gray = clahe->apply(cv_image_gray);

      cv::medianBlur(cv_image_gray, cv_image_gray, 5);

      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners, rejected;
      cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
      cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

      cv::aruco::detectMarkers(cv_image_gray, dictionary, corners, ids, parameters, rejected);

      if (!ids.empty())
      {
        for (size_t i = 0; i < ids.size(); i++)
        {
          cv::Vec3d rvec, tvec;
          cv::aruco::estimatePoseSingleMarkers(corners[i], 0.2, matrix_coefficients, distortion_coefficients, rvec, tvec);

          cv::aruco::drawDetectedMarkers(cv_image, corners);
          cv::aruco::drawAxis(cv_image, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.1);

          cv::Mat rotation_matrix;
          cv::Rodrigues(rvec, rotation_matrix);

          tf::Matrix3x3 tf3d;
          tf3d.setValue(
              rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
              rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
              rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));

          tf::Quaternion quaternion;
          tf3d.getRotation(quaternion);

          if (mode == "P")
          {
            publishPostPose(tvec[0], tvec[1], tvec[2], quaternion);
          }
          else
          {
            accumulatePose(tvec[0], tvec[1], tvec[2], quaternion);
          }
        }

        // Publish "marker detected" message to detected_aruco_image topic
        std_msgs::String msg;
        msg.data = "Marker detected";
        detected_aruco_image_pub_.publish(msg);
      }

      if (mode == "G")
      {
        // publishGatePose(corners);
      }

      drawMarkers(corners, ids, cv_image);

      // cv::imshow("Image window", cv_image);
      // cv::waitKey(3);

      image_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg());

      // Publish to detected_aruco_image topic
      if (!cv_image.empty())
      {
        std_msgs::String msg;
        msg.data = "No marker detected";
        detected_aruco_image_pub_.publish(msg);
      }
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }

  void cameraInfoCallback(const sensor_msgs::CameraInfo &msg)
  {
    distortion_coefficients = msg.D;
    matrix_coefficients = cv::Mat(3, 3, CV_64F, (void *)msg.K.data()).clone();
    camera_info_sub_.shutdown();
  }

private:
  void publishPostPose(double pX, double pY, double pZ, const tf::Quaternion &quaternion)
  {
    postPose.header.frame_id = "camera_realsense_link";
    postPose.pose.position.x = pZ;
    postPose.pose.position.y = pX;
    postPose.pose.position.z = 0;
    postPose.pose.orientation.x = quaternion.x();
    postPose.pose.orientation.y = quaternion.y();
    postPose.pose.orientation.z = quaternion.z();
    postPose.pose.orientation.w = quaternion.w();

    between_pub_.publish(postPose);
    std_msgs::Bool msg;
    msg.data = true;
    AR_pub_.publish(msg);
  }

  void accumulatePose(double pX, double pY, double pZ, const tf::Quaternion &quaternion)
  {
    static double accumulatedX = 0, accumulatedY = 0, accumulatedZ = 0;
    static tf::Quaternion accumulatedQ(0, 0, 0, 1);

    accumulatedX += pX;
    accumulatedY += pY;
    accumulatedZ += pZ;

    accumulatedQ = tf::Quaternion(
        accumulatedQ.x() + quaternion.x(),
        accumulatedQ.y() + quaternion.y(),
        accumulatedQ.z() + quaternion.z(),
        accumulatedQ.w() + quaternion.w());

    publishGatePose(accumulatedX, accumulatedY, accumulatedZ, accumulatedQ);
  }

  void publishGatePose(double pX, double pY, double pZ, const tf::Quaternion &quaternion)
  {
    betweenPose.header.frame_id = "camera_realsense_link";
    betweenPose.pose.position.x = pZ / 2;
    betweenPose.pose.position.y = pX / 2;
    betweenPose.pose.position.z = 0;
    betweenPose.pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(quaternion));

    between_pub_.publish(betweenPose);
    std_msgs::Bool msg;
    msg.data = true;
    AR_pub_.publish(msg);
  }

  void drawMarkers(const std::vector<std::vector<cv::Point2f>> &corners, const std::vector<int> &ids, cv::Mat &image)
  {
    if (!corners.empty())
    {
      for (size_t i = 0; i < corners.size(); i++)
      {
        cv::aruco::drawDetectedMarkers(image, corners, ids);

        cv::Point2f corner1 = corners[i][0];
        cv::Point2f corner2 = corners[i][1];
        cv::Point2f corner3 = corners[i][2];
        cv::Point2f corner4 = corners[i][3];

        cv::line(image, corner1, corner2, cv::Scalar(0, 255, 0), 2);
        cv::line(image, corner2, corner3, cv::Scalar(0, 255, 0), 2);
        cv::line(image, corner3, corner4, cv::Scalar(0, 255, 0), 2);
        cv::line(image, corner4, corner1, cv::Scalar(0, 255, 0), 2);

        cv::Point2f center = (corner1 + corner2 + corner3 + corner4) / 4;
        cv::circle(image, center, 4, cv::Scalar(0, 0, 255), -1);

        cv::putText(image, std::to_string(ids[i]), corner1, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
      }
    }
  }

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber camera_info_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher AR_pub_;
  ros::Publisher between_pub_;
  ros::Publisher detected_aruco_image_pub_; // Publisher for std_msgs::String
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_converter");
  if (argc > 1)
  {
    mode = argv[1];
  }
  // else

  TestNode test_node; // Create an instance of the test node

  ImageConverter ic;
  ros::spin();
  return 0;
}
