#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

class PointCloudProjector {

public:
    PointCloudProjector() : nh_("~"), it_(nh_) {
        std::string cloud_in;
        nh_.param<std::string>("cloud_in", cloud_in, "/camera/depth_registered/points");
        std::string camera_in;
        nh_.param<std::string>("camera_in", camera_in, "/camera/rgb/image_rect_color");
        nh_.param<bool>("use_pixels", image_source_, true);


        ROS_INFO("Using input cloud of %s", cloud_in.c_str());
        ROS_INFO("Using input images of %s", camera_in.c_str());
        if (image_source_)
            ROS_INFO("Using source camera pixels for image creation");
        else
            ROS_INFO("Using point rgb values for the image cration");

        pc_sub_ = nh_.subscribe(cloud_in, 1, &PointCloudProjector::cloud_cb, this);
        camerasub_ = it_.subscribeCamera(camera_in, 1, &PointCloudProjector::image_cb, this);

        imagepub_ = it_.advertise("image_out", 1);

        newimage_ = false;
    }

    void cloud_cb(const sensor_msgs::PointCloud2& input) {
        ROS_INFO_ONCE("Pointcloud received");

        if (last_image_.empty()) {
            ROS_WARN("I don't have an image yet");
            return;
        }
        if (! newimage_)
            return;
        newimage_ = false;

        tf::StampedTransform transform;
        try {
            ros::Duration timeout(1.0 / 30);
            tf_listener_.waitForTransform(cam_model_.tfFrame(), input.header.frame_id,
                                          image_time_, timeout);
            tf_listener_.lookupTransform(cam_model_.tfFrame(), input.header.frame_id,
                                         image_time_, transform);
        }
        catch (tf::TransformException& ex) {
            ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZRGB> camera_cloud;
        pcl::fromROSMsg(input, camera_cloud);
        pcl_ros::transformPointCloud(camera_cloud, camera_cloud, transform);

        cv::Mat newimage(last_image_.size(), CV_8UC3);

        ROS_INFO_ONCE("Using an image size of (%d, %d)", last_image_.rows, last_image_.cols);

        newimage = cv::Scalar(0);

        for (size_t i =0; i<camera_cloud.size(); i++) {
            if (isnan (camera_cloud.points[i].x) || isnan (camera_cloud.points[i].y) || isnan (camera_cloud.points[i].z))
                   continue;
            cv::Point3d pt_cv(camera_cloud.points[i].x, camera_cloud.points[i].y, camera_cloud.points[i].z);
            cv::Point2d uv = cam_model_.project3dToPixel(pt_cv);

            int image_i = int(uv.y);
            int image_j = int(uv.x);

            if (image_i >= last_image_.rows || image_i < 0 || image_j >= last_image_.cols || image_j < 0 ) {
                continue;
            }

            uchar r, g, b;
            //using the source image
            if (image_source_)
            {
                cv::Point3_<uchar>* p = last_image_.ptr<cv::Point3_<uchar> >(image_i, image_j);
                b = p->x;
                g = p->y;
                r = p->z;

//                newimage.at<cv::Vec3b>(uv) = last_image_.at<cv::Vec3b>(uv);
            }
            //using the pointcloud information
            else
            {
                r = camera_cloud.points[i].r;
                g = camera_cloud.points[i].g;
                b = camera_cloud.points[i].b;
            }
            cv::Point3_<uchar>* p = newimage.ptr<cv::Point3_<uchar> >(image_i, image_j);
            p->x = b;
            p->y = g;
            p->z = r;

        }

        std_msgs::Header newheader;
        newheader.frame_id = cam_model_.tfFrame();
        newheader.stamp = image_time_;
        cv_bridge::CvImage cvimage(newheader, "bgr8", newimage);
        imagepub_.publish(cvimage.toImageMsg());

    }

    void image_cb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
        ROS_INFO_ONCE("Image received");

        cv_bridge::CvImagePtr input_bridge;
        try {
            input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
            last_image_ = input_bridge->image;
        }
        catch (cv_bridge::Exception& ex){
          ROS_ERROR("Failed to convert image");
          return;
        }
        cam_model_.fromCameraInfo(info_msg);
        image_time_ = info_msg->header.stamp;
        newimage_ = true;

    }

private:
    ros::Subscriber pc_sub_;
    ros::NodeHandle nh_;
    tf::TransformListener tf_listener_;
    image_transport::CameraSubscriber camerasub_;
    image_transport::ImageTransport it_;
    image_transport::Publisher imagepub_;

    cv::Mat last_image_;
    ros::Time image_time_;
    image_geometry::PinholeCameraModel cam_model_;
    bool newimage_;
    bool image_source_;


};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_projector");
    PointCloudProjector projector;
    ros::spin();
}
