#include <memory>
#include <functional>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>

class RepublisherNode
{
  public:
    RepublisherNode();
    void img_callback(const sensor_msgs::Image::ConstPtr &img_msg)
    {
        cv_bridge::CvImagePtr cvimg = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);

        cv_bridge::CvImage img;
        img.encoding = "mono8";

        cv::cvtColor(cvimg->image, img.image, CV_BGR2GRAY);

        _pub.publish(img.toImageMsg());
    };

  private:
    ros::NodeHandle _nh;
    ros::Publisher _pub;
    ros::Subscriber _sub;
};

RepublisherNode::RepublisherNode()
    : _nh()
{
    _sub = _nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw", 100,
                                             &RepublisherNode::img_callback, this);
    _pub = _nh.advertise<sensor_msgs::Image>("/republisher/image_raw", 100);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "republisher_node");

    RepublisherNode rn;

    ros::spin();

    return 0;
}