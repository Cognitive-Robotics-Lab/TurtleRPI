#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW_IN[] = "Image In";
static const char WINDOW_OUT[] = "Image Out";

class PersonTracker {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

public:
    PersonTracker()
        : it_(nh_) {
        image_pub_ = it_.advertise("image_out", 1);
        image_sub_ = it_.subscribe("image_in", 1, &PersonTracker::image_in_callback, this);

        cv::namedWindow(WINDOW_IN);
        cv::namedWindow(WINDOW_OUT);
    }

    ~PersonTracker() {
        cv::destroyWindow(WINDOW_IN);
        cv::destroyWindow(WINDOW_OUT);
    }

    void image_in_callback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
            cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

        cv::imshow(WINDOW_IN, cv_ptr->image);
        cv::waitKey(3);

        image_pub_.publish(cv_ptr->toImageMsg());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "PersonTracker");
    PersonTracker ic;
    ros::spin();
    return 0;
}
