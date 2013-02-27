#include <ros/ros.h>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/objdetect/objdetect.hpp>

using namespace cv;
namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image";

image_transport::Publisher image_pub_;
CascadeClassifier person_cascade;


void cascade_detect(Mat& frame, CascadeClassifier& cascade) {
    std::vector<Rect> detects;
    Mat frame_gray(frame);

    //cvtColor( frame, frame_gray, CV_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );

    //-- Detect detects
    cascade.detectMultiScale( frame_gray, detects, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );

    for( int i = 0; i < detects.size(); i++ ) {
        Point center( detects[i].x + detects[i].width*0.5, detects[i].y + detects[i].height*0.5 );
        ellipse( frame, center, Size( detects[i].width*0.5, detects[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
    }
}

void image_in_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::BAYER_GRBG8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cascade_detect(cv_ptr->image, person_cascade);
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //    circle(cv_ptr->image, Point(50, 50), 10, CV_RGB(255,0,0));

    imshow(WINDOW, cv_ptr->image);
    waitKey(3);

    image_pub_.publish(cv_ptr->toImageMsg());
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "person_tracker");

    ros::NodeHandle nh_("~");
    image_transport::ImageTransport it_(nh_);
    image_pub_ = it_.advertise("image_out", 1);
    image_transport::Subscriber image_sub_ = it_.subscribe("image_in", 1, image_in_callback);

    std::string cascade_file = "haar/haarcascade_mcs_upperbody.xml";
    if( !(person_cascade.load( cascade_file )) ) {
        ROS_ERROR("--(!)Error loading\n");
        exit(-1);
    };

    namedWindow(WINDOW);

    ROS_INFO("Opencv Version %s", CV_VERSION);

    ros::spin();

    destroyWindow(WINDOW);
    return 0;
}
