/**
 * Example node for reading from the Raspberry PiCam and processing it for
 * AprilTag data.
 */

#include <iostream>

#include <ros/ros.h>

// raspberry pi cam and image handling
#include <raspicam/raspicam_cv.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// apriltag library
extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/tag16h5.h>
#include <apriltag/tag36h11.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/common/homography.h>
}

// topics
#include <geometry_msgs/PoseStamped.h>


/**
 * The node contents in wrapped into a class to allow for easier handling of
 * shared information between the callbacks and main loop
 */
class VisionNode {

public:

	/**
	 * constructor for the vision node
	 * @param frame_width   the integer width of the image frame in pixels
	 * @param frame_height  the integer height of the image frame in pixels
	 * @param publish_image boolean flag for whether or not to publish the images
	 */
	VisionNode(int frame_width, int frame_height, bool publish_image);


	/**
	 * the main loop to be run for this node (called by the `main` function)
	 * @return exit code
	 */
	int run();


private:

	// node handler and image transport handler
	ros::NodeHandle _nh;
	image_transport::ImageTransport _it;


	// settings, etc
	int _frame_width;		// image width to use in [px]
	int _frame_height;		// image height to use in [px]
	bool _publish_image;	// true if the image data should be published

	// camera stuff
	raspicam::RaspiCam_Cv _camera;	// the camera object

	// publishers
	ros::Publisher _tag_relative_position_pub;	// the relative position vector to the truck (NOT IMPLEMENTED)
	ros::Publisher _tag_details_pub;			// the raw tag details (for debugging) (NOT IMPLEMENTED)
	image_transport::Publisher _image_pub;		// the raw annotated image (for debugging)
};


VisionNode::VisionNode(int frame_width, int frame_height, bool publish_image) :
_frame_width(frame_width),
_frame_height(frame_height),
_it(_nh)
{
	// publishers
	_image_pub = _it.advertise("image", 1);	// NOTE: should not be used in flight
	_tag_relative_position_pub = _nh.advertise<geometry_msgs::PoseStamped>("landing_pose", 1);	// EXAMPLE publishing


    // configure the camera
    _camera.set(cv::CAP_PROP_FORMAT, CV_8UC1);				// 8 bit image data -> means grayscale image
    _camera.set(cv::CAP_PROP_FRAME_WIDTH, _frame_width);	// set the width of the image
	_camera.set(cv::CAP_PROP_FRAME_HEIGHT, _frame_height);	// set the height of the image
}


int VisionNode::run() {

    // open the camera
    ROS_INFO("opening camera");
	if (!_camera.open()) {
        ROS_ERROR("Error opening the camera");
        std::cerr << "Error opening the camera" << std::endl;
        return -1;
    }

    // apriltag handling setup
    // see readme for details: https://github.com/AprilRobotics/apriltag
	apriltag_family_t *tf = tag16h5_create();	// specify the tag family size:0.09
    //apriltag_family_t *tf = tag36h11_create();  // specify the tag family size:0.2

	// initialize the detector and some attributes for detection
	apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = 3.0;
    td->quad_sigma = 0.0;
    td->refine_edges = 0;
    //td->decode_sharpening = 0.25;
    
	ros::Time image_time; 	// timestamp of when the image was grabbed
	cv::Mat frame_gray;		// the image in grayscale

	// loop forever while the node should be running
	while (ros::ok()) {

		// grab the frame from the camera
		// see readme for details: https://github.com/cedricve/raspicam
        _camera.grab();
		_camera.retrieve(frame_gray);
		image_time = ros::Time::now();

        // Make an image_u8_t header for the Mat data to pass over to the april
        // tag detector
        image_u8_t im = { .width = frame_gray.cols,
            .height = frame_gray.rows,
            .stride = frame_gray.cols,
            .buf = frame_gray.data
        };

        // run the detector
		zarray_t *detections = apriltag_detector_detect(td, &im);
        ROS_INFO("%d tags detected", zarray_size(detections));

        // NOTE: this is where the relative vector should be computed

        // if flagged to do so, annotate and publish the image
		if (_publish_image) {
            // Draw detection outlines
            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);
                line(frame_gray, cv::Point(det->p[0][0], det->p[0][1]),
                         cv::Point(det->p[1][0], det->p[1][1]),
                         cv::Scalar(0, 0xff, 0), 2);
                line(frame_gray, cv::Point(det->p[0][0], det->p[0][1]),
                         cv::Point(det->p[3][0], det->p[3][1]),
                         cv::Scalar(0, 0, 0xff), 2);
                line(frame_gray, cv::Point(det->p[1][0], det->p[1][1]),
                         cv::Point(det->p[2][0], det->p[2][1]),
                         cv::Scalar(0xff, 0, 0), 2);
                line(frame_gray, cv::Point(det->p[2][0], det->p[2][1]),
                         cv::Point(det->p[3][0], det->p[3][1]),
                         cv::Scalar(0xff, 0, 0), 2);

                std::stringstream ss;
                ss << det->id;
                cv::String text = ss.str();
                int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
                double fontscale = 1.0;
                int baseline;
                cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2,
                                                &baseline);
                cv::putText(frame_gray, text, cv::Point(det->c[0]-textsize.width/2,
                                           det->c[1]+textsize.height/2),
                        fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);

                // publish the distance
                // First create an apriltag_detection_info_t struct using your known parameters.
                apriltag_detection_info_t info;
                info.det = det;
                info.tagsize = 0.09;
                info.fx = 478.2245; //1276.7 or 1842.127 or 981.1 for test not real
                info.fy = 478.2245; //1274.34
                info.cx = 320.0;
                info.cy = 240.0;

                // Then call estimate_tag_pose.
                apriltag_pose_t pose;
                double err = estimate_tag_pose(&info, &pose);
                float lx = pose.t->data[0];
                float ly = pose.t->data[1];
                float lz = pose.t->data[2];

                std::cout << "Px:" << lx << "Py:" << ly << "Pz:" << lz;   
                geometry_msgs::PoseStamped res;
                res.header.stamp = ros::Time::now();
                res.pose.position.x = lx;
                res.pose.position.y = ly;
                res.pose.position.z = lz;
                _tag_relative_position_pub.publish(res);

            }

			// publish the annotated image
			std_msgs::Header header;
			header.stamp = image_time;
			sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "mono8", frame_gray).toImageMsg();
			_image_pub.publish(img_msg);

		}

        // clean up the detections
        zarray_destroy(detections);

        // call spin once to trigger any callbacks
        // while there are none specified so far, it's good practice to just
        // throw this in so if/when you add callbacks you don't spend hours
        // trying to figure out why they aren't being called!
		ros::spinOnce();
	}

    // need to stop the camera
    ROS_INFO("stopping camera");
	_camera.release();

    // remove apriltag stuff
	apriltag_detector_destroy(td);
    //tag36h11_destroy(tf);
	tag16h5_destroy(tf);

}


// the main function


int main(int argc, char **argv) {

	// initialize th enode
	ros::init(argc, argv, "vision_node");

	// get parameters from the launch file which define some camera settings
	ros::NodeHandle private_nh("~");
	int frame_width, frame_height;
	bool publish_image;
	private_nh.param("frame_width", frame_width, 640);
	private_nh.param("frame_height", frame_height, 480);
	private_nh.param("publish_image", publish_image, false);

	// create the node
	VisionNode node(frame_width, frame_height, publish_image);

	// run the node
	return node.run();
}
