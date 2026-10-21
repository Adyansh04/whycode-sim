// OpenCV's ArUco detector on AprilTag 36h11 tags, a baseline for the benchmark. Each tag's pose
// goes out on /tf from the image's frame to "aruco:<id>", as apriltag_ros publishes its tags,
// so the same scorer reads both.
//
//   ros2 run whycode_baselines aruco_node --ros-args -r image:=/camera/image_raw
//       -r camera_info:=/camera/camera_info -p ids:="[0, 1]" -p sizes:="[0.2, 0.146]"
//
// `size` is the black square's edge (m) for ids without their own entry in `ids`/`sizes`.

#include <algorithm>
#include <array>
#include <cmath>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <tf2_msgs/msg/tf_message.hpp>
#include <unordered_map>
#include <vector>

namespace
{
// A rotation matrix's quaternion (x, y, z, w)
std::array<double, 4> toQuaternion(const cv::Matx33d& r)
{
    const double trace = r(0, 0) + r(1, 1) + r(2, 2);
    if (trace > 0.0)
    {
        const double s = 0.5 / std::sqrt(trace + 1.0);
        return { (r(2, 1) - r(1, 2)) * s,
                 (r(0, 2) - r(2, 0)) * s,
                 (r(1, 0) - r(0, 1)) * s,
                 0.25 / s };
    }
    if (r(0, 0) > r(1, 1) && r(0, 0) > r(2, 2))
    {
        const double s = 2.0 * std::sqrt(1.0 + r(0, 0) - r(1, 1) - r(2, 2));
        return { 0.25 * s,
                 (r(0, 1) + r(1, 0)) / s,
                 (r(0, 2) + r(2, 0)) / s,
                 (r(2, 1) - r(1, 2)) / s };
    }
    if (r(1, 1) > r(2, 2))
    {
        const double s = 2.0 * std::sqrt(1.0 + r(1, 1) - r(0, 0) - r(2, 2));
        return { (r(0, 1) + r(1, 0)) / s,
                 0.25 * s,
                 (r(1, 2) + r(2, 1)) / s,
                 (r(0, 2) - r(2, 0)) / s };
    }
    const double s = 2.0 * std::sqrt(1.0 + r(2, 2) - r(0, 0) - r(1, 1));
    return { (r(0, 2) + r(2, 0)) / s, (r(1, 2) + r(2, 1)) / s, 0.25 * s, (r(1, 0) - r(0, 1)) / s };
}
}  // namespace

class ArucoNode : public rclcpp::Node
{
public:
    ArucoNode()
      : Node("aruco_detector")
    {
        default_size_     = declare_parameter("size", 0.2);
        const auto ids    = declare_parameter("ids", std::vector<int64_t>{});
        const auto sizes  = declare_parameter("sizes", std::vector<double>{});
        const auto refine = declare_parameter("corner_refinement", std::string("subpix"));
        for (size_t i = 0; i < std::min(ids.size(), sizes.size()); i++)
        {
            sizes_[static_cast<int>(ids[i])] = sizes[i];
        }

        parameters_                         = cv::aruco::DetectorParameters::create();
        parameters_->cornerRefinementMethod = refine == "apriltag" ?
                                                  cv::aruco::CORNER_REFINE_APRILTAG :
                                              refine == "none" ? cv::aruco::CORNER_REFINE_NONE :
                                                                 cv::aruco::CORNER_REFINE_SUBPIX;
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);

        // Every frame of a bag replay, as the WhyCode node takes them
        const auto qos =
            rclcpp::QoS(static_cast<size_t>(declare_parameter("queue_size", 30))).reliable();
        tf_   = create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));
        info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info",
            qos,
            [this](const sensor_msgs::msg::CameraInfo& info) {
                camera_ = cv::Matx33d(info.k.data());
                distortion_.assign(info.d.begin(), info.d.end());
            });
        image_ = create_subscription<sensor_msgs::msg::Image>(
            "image",
            qos,
            [this](const sensor_msgs::msg::Image& image) { onImage(image); });
        RCLCPP_INFO(
            get_logger(),
            "ArUco DICT_APRILTAG_36h11, %s corner refinement, default size %.3f m",
            refine.c_str(),
            default_size_);
    }

private:
    void onImage(const sensor_msgs::msg::Image& image)
    {
        if (camera_(0, 0) <= 0.0)
        {
            return;  // No intrinsics yet
        }
        const int     code = image.encoding == "bgr8"  ? cv::COLOR_BGR2GRAY :
                             image.encoding == "mono8" ? -1 :
                                                         cv::COLOR_RGB2GRAY;
        const cv::Mat frame(
            static_cast<int>(image.height),
            static_cast<int>(image.width),
            code < 0 ? CV_8UC1 : CV_8UC3,
            const_cast<uint8_t*>(image.data.data()),
            image.step);
        if (code < 0)
        {
            gray_ = frame;
        }
        else
        {
            cv::cvtColor(frame, gray_, code);
        }

        corners_.clear();
        ids_.clear();
        cv::aruco::detectMarkers(gray_, dictionary_, corners_, ids_, parameters_);

        transforms_.transforms.clear();
        for (size_t i = 0; i < ids_.size(); i++)
        {
            const auto   found = sizes_.find(ids_[i]);
            const double half  = 0.5 * (found == sizes_.end() ? default_size_ : found->second);
            // ArUco's corner order: top-left, top-right, bottom-right, bottom-left
            const std::array<cv::Point3d, 4> square{ cv::Point3d(-half, half, 0),
                                                     cv::Point3d(half, half, 0),
                                                     cv::Point3d(half, -half, 0),
                                                     cv::Point3d(-half, -half, 0) };
            cv::Vec3d                        rotation, translation;
            if (!cv::solvePnP(
                    square,
                    corners_[i],
                    camera_,
                    distortion_,
                    rotation,
                    translation,
                    false,
                    cv::SOLVEPNP_IPPE_SQUARE))
            {
                continue;
            }
            cv::Matx33d matrix;
            cv::Rodrigues(rotation, matrix);
            const auto quaternion = toQuaternion(matrix);

            geometry_msgs::msg::TransformStamped transform;
            transform.header                  = image.header;
            transform.child_frame_id          = "aruco:" + std::to_string(ids_[i]);
            transform.transform.translation.x = translation[0];
            transform.transform.translation.y = translation[1];
            transform.transform.translation.z = translation[2];
            transform.transform.rotation.x    = quaternion[0];
            transform.transform.rotation.y    = quaternion[1];
            transform.transform.rotation.z    = quaternion[2];
            transform.transform.rotation.w    = quaternion[3];
            transforms_.transforms.push_back(transform);
        }
        if (!transforms_.transforms.empty())
        {
            tf_->publish(transforms_);
        }
    }

    double                                 default_size_;
    std::unordered_map<int, double>        sizes_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;
    cv::Ptr<cv::aruco::Dictionary>         dictionary_;
    cv::Matx33d                            camera_ = cv::Matx33d::zeros();
    std::vector<double>                    distortion_;
    cv::Mat                                gray_;
    std::vector<std::vector<cv::Point2f>>  corners_;
    std::vector<int>                       ids_;
    tf2_msgs::msg::TFMessage               transforms_;

    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr        tf_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      image_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoNode>());
    rclcpp::shutdown();
    return 0;
}
