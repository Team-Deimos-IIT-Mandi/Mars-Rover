#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class ArucoDetector
{
public:
    ArucoDetector()
    {
        // Initialize the subscriber to the image topic
        image_sub_ = nh_.subscribe("/camera/image_raw", 1, &ArucoDetector::imageCallback, this);

        // Load the predefined dictionary
        aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        parameters_ = cv::aruco::DetectorParameters::create();
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        // Convert the ROS image message to a CV image
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Detect ArUco markers
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;
        cv::aruco::detectMarkers(cv_ptr->image, aruco_dict_, corners, ids, parameters_, rejected);

        // Draw detected markers on the image
        if (!ids.empty())
        {
            cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
        }

        // Display the image with detected markers
        cv::imshow("Detected ArUco markers", cv_ptr->image);
        cv::waitKey(1);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_detector_cpp");
    ArucoDetector aruco_detector;

    ros::spin();
    cv::destroyAllWindows();
    return 0;
}
