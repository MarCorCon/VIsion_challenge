
#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>

using namespace std;
//std::vector<int> seconds;
//std::vector<int> nanoseconds;
//std::vector<int> x_vector;
//std::vector<int> y_vector;
//ofstream groundtruth("/home/mcc/catkin_ws/src/vision_challenge/src/groundtruth.txt");
ofstream groundtruth("src/vision_challenge/src/result/groundtruth.txt");

void searchTemplate(cv::Mat &in, cv::Mat &temp, cv::Mat &out) {

    double min, max;

    cv::Point pointMin, pointMax;
    
    cv::Point pointMaxGlobal;

   cv::matchTemplate(in, temp, out, CV_TM_CCOEFF_NORMED);
   cv::minMaxLoc(out, &min, &max, &pointMin, &pointMaxGlobal);

   
    in.copyTo(out);

    //Draw a circle on the output image at the location of the maxima, and a rectangle with the dimensions of the template temp
    cv::circle(out, pointMaxGlobal, 10, cv::Scalar(0, 0, 255), 3);
   // cv::rectangle(out, pointMax, cv::Point(pointMaxGlobal.x + selected_template.cols, pointMaxGlobal.y + selected_template.rows), cv::Scalar(0, 0, 255), 3);
   groundtruth<< pointMaxGlobal.x<<"\t"<<pointMaxGlobal.y<<"\n";

}

void imageCb(const sensor_msgs::ImageConstPtr & msg) {
    cv::Mat rgb_frame; //Input image in matrix form
    cv::Mat out_frame; //Output image
    groundtruth<<msg->header.stamp.sec<<"\t"<<msg->header.stamp.nsec<<"\t";
    
    cv_bridge::CvImagePtr cv_ptr_rgb; //cv_bridge::CvImagePtr is a pointer type that points to NULL by default. You have to allocate storage before you can actually use it
    try {
        cv_ptr_rgb = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //make a copy of the ROS message data. //bgra8: BGR color image with an alpha channel
        //Note that mono8 and bgr8 are the two image encodings expected by most OpenCV functions.
    }    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    rgb_frame = cv_ptr_rgb->image; //Here we have the current frame in OpenCV Mat format
    //cv::Mat template_image = cv::imread("/home/mcc/catkin_ws/src/vision_challenge/src/green_red_2.jpg");
    cv::Mat template_image = cv::imread("src/vision_challenge/src/templates/green_red_2.jpg");
    //processImageColor_c4(rgb_frame,out_frame);
    searchTemplate(rgb_frame, template_image, out_frame);
    //cv::imshow("input_image", rgb_frame); //Show a window with the input image
    cv::imshow("output_image", out_frame); //Show a window with the output image
    
    cv::waitKey(3); //Wait for 3 milliseconds
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vc_main");

    ros::NodeHandle nh_;

    image_transport::ImageTransport it_(nh_);

    image_transport::Subscriber image_sub_;
    
    std::string in1, out1;
    int blockNumber = 0;
    image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 1, imageCb); //subscribes to the Kinect video frames
    
    ros::spin();

    return 0;
}
