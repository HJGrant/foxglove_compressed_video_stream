#include <stdio.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <foxglove_msgs/msg/compressed_video.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>

class oakd_video_stream : public rclcpp::Node
{
private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr oak_image_sub;
  rclcpp::Publisher<foxglove_msgs::msg::CompressedVideo>::SharedPtr write_video_stream;
  GstElement* appsrc_;
  GstElement* appsink_;
  bool caps_set_ = false;

public:
    oakd_video_stream(const rclcpp::NodeOptions & options)
    : Node("video_stream_node")
    {
  
    RCLCPP_INFO(this->get_logger(), "Initialising CompressedVideo Stream!");

    oak_image_sub = this->create_subscription<sensor_msgs::msg::Image>(
    "/image_raw",
    rclcpp::SensorDataQoS(),
    std::bind(&oakd_video_stream::writeToStreamCallback, this, std::placeholders::_1));
    
    write_video_stream = this->create_publisher<foxglove_msgs::msg::CompressedVideo>("/compressed_video", 10);
    
    gst_init(nullptr, nullptr);
    std::string pipeline_desc =
        "appsrc name=mysrc ! videoconvert ! nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! h264parse ! appsink name=mysink";
    GstElement* pipeline = gst_parse_launch(pipeline_desc.c_str(), nullptr);
    appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline), "mysrc");
    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline), "mysink");

    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    }

private:

    void writeToStreamCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    
      // 1. Convert ROS image to cv::Mat
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    cv::Mat frame = cv_ptr->image;

    if (!caps_set_) {
        GstCaps* caps = gst_caps_new_simple(
            "video/x-raw",
            "format", G_TYPE_STRING, "BGR",
            "width", G_TYPE_INT, frame.cols,
            "height", G_TYPE_INT, frame.rows,
            "framerate", GST_TYPE_FRACTION, 30, 1, // or your actual framerate
            NULL
        );
        g_object_set(G_OBJECT(appsrc_), "caps", caps, NULL);
        gst_caps_unref(caps);
        caps_set_ = true;
    }

    //RCLCPP_INFO(this->get_logger(), "cv::Mat type: %d, channels: %d, depth: %d", frame.type(), frame.channels(), frame.depth());

    // 2. Push frame to GStreamer pipeline (assume pipeline/appsrc/appsink set up in constructor)
    GstBuffer* buffer = gst_buffer_new_allocate(nullptr, frame.total() * frame.elemSize(), nullptr);
    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_WRITE);
    memcpy(map.data, frame.data, frame.total() * frame.elemSize());
    gst_buffer_unmap(buffer, &map);

    GstFlowReturn ret;
    g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);

    // 3. Pull encoded H.264 frame from appsink
    GstSample* sample = nullptr;
    g_signal_emit_by_name(appsink_, "try-pull-sample", (guint64)0, &sample);
    if (!sample) {
        RCLCPP_WARN(this->get_logger(), "No sample received from appsink!");
        return;
    }

    GstBuffer* enc_buf = gst_sample_get_buffer(sample);
    gst_buffer_map(enc_buf, &map, GST_MAP_READ);

    // 4. Fill CompressedVideo message
    foxglove_msgs::msg::CompressedVideo video_msg;
    video_msg.timestamp = msg->header.stamp;
    video_msg.frame_id = msg->header.frame_id;
    video_msg.format = "h264";
    video_msg.data.assign(map.data, map.data + map.size);

    gst_buffer_unmap(enc_buf, &map);
    gst_sample_unref(sample);

    RCLCPP_INFO(this->get_logger(), "Pushed Frame to Stream!");
    write_video_stream->publish(video_msg);
}

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<oakd_video_stream>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
