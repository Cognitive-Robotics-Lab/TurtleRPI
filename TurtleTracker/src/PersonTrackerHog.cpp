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
HOGDescriptor person_hog;


void hog_detect(Mat& frame, HOGDescriptor& hog) {
    Mat frame_gray(frame);

    //cvtColor( frame, frame_gray, CV_BGR2GRAY );
    //equalizeHist( frame_gray, frame_gray );

    // // run the detector with default parameters. to get a higher hit-rate
    // // (and more false alarms, respectively), decrease the hitThreshold and
    // // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).
    // // std::vector<Rect> found, found_filtered;
    // hog.detectMultiScale(frame, found, 0, Size(8,8), Size(32,32), 1.05, 2);

    // size_t i, j;
    // for( i = 0; i < found.size(); i++ ) {
    //     Rect r = found[i];
    //     for( j = 0; j < found.size(); j++ )
    //         if( j != i && (r & found[j]) == r)
    //             break;
    //     if( j == found.size() )
    //         found_filtered.push_back(r);
    // }
    // for( i = 0; i < found_filtered.size(); i++ ) {
    //     Rect r = found_filtered[i];
    //     // the HOG detector returns slightly larger rectangles than the real objects.
    //     // so we slightly shrink the rectangles to get a nicer output.
    //     r.x += cvRound(r.width*0.1);
    //     r.width = cvRound(r.width*0.8);
    //     r.y += cvRound(r.height*0.07);
    //     r.height = cvRound(r.height*0.8);
    //     rectangle(frame, r.tl(), r.br(), cv::Scalar(0,255,0), 3);
    // }

    // run the detector with default parameters. to get a higher hit-rate
    // (and more false alarms, respectively), decrease the hitThreshold and
    // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).
    std::vector<Point> found, found_filtered;
    hog.detect(frame, found);

    for( int i = 0; i < found.size(); i++ ) {
        ellipse( frame, found[i], Size(50,50), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
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

    hog_detect(cv_ptr->image, person_hog);
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

    person_hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());

    namedWindow(WINDOW);

    ROS_INFO("Opencv Version %s", CV_VERSION);

    ros::spin();

    destroyWindow(WINDOW);
    return 0;
}
