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
        ROS_INFO("Using input cloud of %s", cloud_in.c_str());
        ROS_INFO("Using input images of %s", camera_in.c_str());

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
        newimage = cv::Scalar(0);

        int tot_points = 0;
        int filled_points = 0;
        for (size_t i =0; i<camera_cloud.size(); i++) {
            tot_points++;
            if (isnan (camera_cloud.points[i].x) || isnan (camera_cloud.points[i].y) || isnan (camera_cloud.points[i].z))
                   continue;
            cv::Point3d pt_cv(camera_cloud.points[i].x, camera_cloud.points[i].y, camera_cloud.points[i].z);
            cv::Point2d uv = cam_model_.project3dToPixel(pt_cv);
            int image_i = int(uv.y);
            int image_j = int(uv.x);
            if (image_i >= newimage.rows || image_i < 0 || image_j >= newimage.cols || image_j < 0 ) {
                continue;
            }
//            ROS_INFO("Accessing point (%d, %d)", image_i, image_j);
            cv::Point3_<uchar>* p = newimage.ptr<cv::Point3_<uchar> >(image_i, image_j);
            p->x = camera_cloud.points[i].r;
            p->y = camera_cloud.points[i].g;
            p->z = camera_cloud.points[i].b;
//            uchar p0 = newimage.at<cv::Vec3b>(uv)[0];
//            uchar p1 = newimage.at<cv::Vec3b>(uv)[1];
//            uchar p2 = newimage.at<cv::Vec3b>(uv)[2];
//            uchar p0 = camera_cloud.points[i].r;
//            uchar p1 = camera_cloud.points[i].g;
//            uchar p2 = camera_cloud.points[i].b;

//            ROS_INFO("%d %d %d", p0, p1, p2);
            filled_points++;

        }

        std_msgs::Header newheader;
        newheader.frame_id = cam_model_.tfFrame();
        newheader.stamp = image_time_;
        cv_bridge::CvImage cvimage(newheader, "rgb8", newimage);
        imagepub_.publish(cvimage.toImageMsg());
        ROS_INFO("Filled %d points out of %d", filled_points, tot_points);

    }

    void image_cb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
        ROS_INFO_ONCE("Image received");

        cv_bridge::CvImagePtr input_bridge;
        try {
            input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
            last_image_ = input_bridge->image;
        }
        catch (cv_bridge::Exception& ex){
          ROS_ERROR("[draw_frames] Failed to convert image");
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


};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_projector");
    PointCloudProjector projector;
    ros::spin();
}
