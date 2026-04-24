#include <cmath>
#include <cstring>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class CropPointcloud : public rclcpp::Node
{
public:
  CropPointcloud() : Node("crop_pointcloud")
  {
    declare_parameter("input_topic", "/camera/depth/color/points");
    declare_parameter("output_topic", "/camera/depth/color/points_cropped");
    declare_parameter("reference_frame", "world");
    declare_parameter("range", 0.6);
    declare_parameter("publish_rate", 5.0);

    auto input_topic  = get_parameter("input_topic").as_string();
    auto output_topic = get_parameter("output_topic").as_string();
    ref_frame_ = get_parameter("reference_frame").as_string();
    range_     = get_parameter("range").as_double();
    double rate = get_parameter("publish_rate").as_double();
    min_interval_ns_ = (rate > 0.0) ? static_cast<int64_t>(1e9 / rate) : 0;

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::SensorDataQoS qos;
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, qos);
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, qos,
        std::bind(&CropPointcloud::callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Cropping '%s' -> '%s' within %.2fm of '%s' at %.1f Hz",
                input_topic.c_str(), output_topic.c_str(), range_, ref_frame_.c_str(), rate);
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto now = get_clock()->now().nanoseconds();
    if (now - last_publish_ns_ < min_interval_ns_) return;

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(ref_frame_, msg->header.frame_id,
                                       tf2::TimePointZero);
    } catch (...) {
      return;
    }

    const auto &t = tf.transform.translation;
    const auto &q = tf.transform.rotation;
    // Rotation matrix from quaternion
    const double r00 = 1.0 - 2.0*(q.y*q.y + q.z*q.z);
    const double r01 = 2.0*(q.x*q.y - q.z*q.w);
    const double r02 = 2.0*(q.x*q.z + q.y*q.w);
    const double r10 = 2.0*(q.x*q.y + q.z*q.w);
    const double r11 = 1.0 - 2.0*(q.x*q.x + q.z*q.z);
    const double r12 = 2.0*(q.y*q.z - q.x*q.w);
    const double r20 = 2.0*(q.x*q.z - q.y*q.w);
    const double r21 = 2.0*(q.y*q.z + q.x*q.w);
    const double r22 = 1.0 - 2.0*(q.x*q.x + q.y*q.y);

    // Find x, y, z field offsets
    int x_off = -1, y_off = -1, z_off = -1;
    for (const auto &f : msg->fields) {
      if (f.name == "x") x_off = static_cast<int>(f.offset);
      else if (f.name == "y") y_off = static_cast<int>(f.offset);
      else if (f.name == "z") z_off = static_cast<int>(f.offset);
    }
    if (x_off < 0 || y_off < 0 || z_off < 0) return;

    const uint32_t stride = msg->point_step;
    const uint32_t n_points = msg->width * msg->height;
    const uint8_t *src = msg->data.data();
    const double R = range_;

    // Pre-allocate output buffer (worst case = all points kept)
    std::vector<uint8_t> out_data;
    out_data.reserve(msg->data.size());
    uint32_t kept = 0;

    for (uint32_t i = 0; i < n_points; ++i) {
      const uint8_t *pt = src + i * stride;

      float lx, ly, lz;
      std::memcpy(&lx, pt + x_off, sizeof(float));
      std::memcpy(&ly, pt + y_off, sizeof(float));
      std::memcpy(&lz, pt + z_off, sizeof(float));

      if (!std::isfinite(lx)) continue;

      // Transform to reference frame
      const double wx = r00*lx + r01*ly + r02*lz + t.x;
      const double wy = r10*lx + r11*ly + r12*lz + t.y;
      const double wz = r20*lx + r21*ly + r22*lz + t.z;

      if (std::abs(wx) < R && std::abs(wy) < R && std::abs(wz) < R) {
        out_data.insert(out_data.end(), pt, pt + stride);
        ++kept;
      }
    }

    if (kept == 0) return;

    auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();
    out->header = msg->header;
    out->height = 1;
    out->width  = kept;
    out->fields = msg->fields;
    out->is_bigendian = msg->is_bigendian;
    out->point_step   = stride;
    out->row_step     = stride * kept;
    out->is_dense     = true;
    out->data = std::move(out_data);
    pub_->publish(std::move(out));
    last_publish_ns_ = now;
  }

  std::string ref_frame_;
  double range_;
  int64_t min_interval_ns_{0};
  int64_t last_publish_ns_{0};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CropPointcloud>());
  rclcpp::shutdown();
  return 0;
}
