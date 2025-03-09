#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include "MLX9064X_I2C_Driver.h"
#include "MLX90640_API.h"

class MLX90640Node : public rclcpp::Node{
  public:
    MLX90640Node(): Node("mlx90640_node"){
      RCLCPP_INFO(this->get_logger(), "Initializing MLX90640Node...");

      // Declare and get parameters
      this->declare_parameter("camera.i2c_address", 0x33);
      this->declare_parameter("camera.refresh_rate", 8);
      this->declare_parameter("camera.emissivity", 0.95);
      this->declare_parameter("camera.ambient_temperature", 23.0);

      i2c_address_ = this->get_parameter("camera.i2c_address").as_int();
      refresh_rate_ = this->get_parameter("camera.refresh_rate").as_int();
      emissivity_ = this->get_parameter("camera.emissivity").as_double();
      ambient_temperature_ = this->get_parameter("camera.ambient_temperature").as_double();

      thermal_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/thermal_image", 10);
      average_temp_pub_ = this->create_publisher<std_msgs::msg::Float32>("/average_temperature", 10);
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / refresh_rate_), std::bind(&MLX90640Node::timer_callback, this));

      // Initialize the MLX90640
      RCLCPP_INFO(this->get_logger(), "Initializing I2C...");
      MLX9064x_I2CInit();

      RCLCPP_INFO(this->get_logger(), "Setting refresh rate...");
      MLX90640_SetRefreshRate(i2c_address_, 0x03); // Set refresh rate to 4Hz

      RCLCPP_INFO(this->get_logger(), "Dumping EEPROM data...");
      MLX90640_DumpEE(i2c_address_, eeData);

      RCLCPP_INFO(this->get_logger(), "Extracting parameters...");
      MLX90640_ExtractParameters(eeData, &mlx90640);

      RCLCPP_INFO(this->get_logger(), "MLX90640Node initialized successfully.");
    }

  private:
    void timer_callback(){
      uint16_t frameData[834];
      float image[768];
      float average_temp = 0.0;

      // Read frame data
      RCLCPP_INFO(this->get_logger(), "Reading frame data...");
      MLX90640_GetFrameData(i2c_address_, frameData);
      MLX90640_CalculateTo(frameData, &mlx90640, emissivity_, MLX90640_GetTa(frameData, &mlx90640) - ambient_temperature_, image);

      // Calculate average temperature
      for (int i = 0; i < 768; i++) {
        average_temp += image[i];
      }
      average_temp /= 768;
      RCLCPP_INFO(this->get_logger(), "Average temperature: %f", average_temp);

      // Create and publish the thermal image message
      sensor_msgs::msg::Image thermal_image;
      thermal_image.header.stamp = this->now();
      thermal_image.header.frame_id = "thermal_image";
      thermal_image.height = 24;
      thermal_image.width = 32;
      thermal_image.encoding = "32FC1";
      thermal_image.is_bigendian = false;
      thermal_image.step = 32 * sizeof(float);
      thermal_image.data.resize(32 * 24 * sizeof(float));
      memcpy(thermal_image.data.data(), image, 32 * 24 * sizeof(float));
      thermal_image_pub_->publish(thermal_image);
      RCLCPP_INFO(this->get_logger(), "Published thermal image");

      // Create and publish the average temperature message
      std_msgs::msg::Float32 avg_temp_msg;
      avg_temp_msg.data = average_temp;
      average_temp_pub_->publish(avg_temp_msg);
      RCLCPP_INFO(this->get_logger(), "Published average temperature");
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr thermal_image_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr average_temp_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    paramsMLX90640 mlx90640;
    uint16_t eeData[832];
    int i2c_address_;
    int refresh_rate_;
    double emissivity_;
    double ambient_temperature_;
};

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MLX90640Node>());
  rclcpp::shutdown();
  return 0;
}
