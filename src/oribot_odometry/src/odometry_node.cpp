#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include <JetsonGPIO.h>
#include <vector>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono;
using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;
using namespace tf2_msgs::msg;


static const float WHEEL_RADIUS = 65.0/2.0/1000.0;
static const float WHEEL_CIRCUMFERENCE = 2 * M_PI * WHEEL_RADIUS;
static const float RADIANS = M_PI/180.0;
static const float MAX_RADIANS = 2*M_PI;
static const float TICKS_PER_REVOLUTION = 360;

static const float WHEEL_SEPARATION_X = 171.0/1000.0;
static const float WHEEL_SEPARATION_Y = 185.0/1000.0;


struct QT{
  float x;
  float y;
  float z;
  float w;
};

struct odom_data_t {
  float linear_x;
  float linear_y;
  float angular_z;
  float pose_x;
  float pose_y;
  float orientation_z;
};


static void quarternion(QT& qt, float x, float y, float z) {
  float cosx = cos(x/2.0);
  float sinx = sin(x/2.0);
  float cosy = cos(y/2.0);
  float siny = sin(y/2.0);
  float cosz = cos(z/2.0);
  float sinz = sin(z/2.0);
  qt.x = sinx * cosy * cosz - cosx * siny * sinz;
  qt.y = cosx * siny * cosz + sinx * cosy * sinz;
  qt.z = cosx * cosy * sinz - sinx * siny * cosz;
  qt.w = cosx * cosy * cosz + sinx * siny * sinz;
}

class TicksCounter {
  public:
    int ticks;
    int start_ticks;
    int last_enca_state;
    int enca_;
    int encb_;
    int orientation_;
    float rpm;
    float previous_rpm;
    float average_rpm;
    float velocity;
    float previous_velocity;
    float average_velocity;

    TicksCounter(int enca, int encb, int orientation) : enca_(enca), encb_(encb), orientation_(orientation) {
      GPIO::setmode(GPIO::BOARD);
      last_enca_state = 0;
      ticks = 0;
      start_ticks = 0;
      rpm = 0;
      previous_rpm = 0;
      average_rpm = 0;
      velocity = 0;
      previous_velocity = 0;
      average_velocity = 0;



      GPIO::setup(enca_, GPIO::IN);
      GPIO::setup(encb_, GPIO::IN);

    }

    void enca_callback(const std::string& channel) {
      int encb_state = GPIO::input(encb_);
      if(last_enca_state != encb_state) {
        ticks += 1;
      } else {
        ticks -= 1;
      }
      last_enca_state = encb_state;
    }
    
    void cleanup() {
      GPIO::cleanup(enca_);
      GPIO::cleanup(encb_);
    }

    ~TicksCounter(){
      cleanup();
    }

    void set_rpm(float rpm_value) {
      previous_rpm = rpm;
      rpm = rpm_value * orientation_;
      average_rpm = (rpm + previous_rpm) / 2.0;
      previous_velocity = velocity;
      velocity = rpm * WHEEL_CIRCUMFERENCE / 60.0;
      average_velocity = (velocity + previous_velocity) / 2.0;
    }

    
   

};

static TicksCounter * tick1 = new TicksCounter(18,16,-1);
static TicksCounter * tick2 = new TicksCounter(24,22,1);
static TicksCounter * tick3 = new TicksCounter(15,13,-1);
static TicksCounter * tick4 = new TicksCounter(11,7,1);

static void process_ticks(TicksCounter * ticker) {
  int encb_state = GPIO::input(ticker->encb_);
  if(ticker->last_enca_state != encb_state) {
    ticker->ticks += 1;
  } else {
    ticker->ticks -= 1;
  }
  ticker->last_enca_state = encb_state;
}

static void tick1_callback(const std::string& channel) {
  process_ticks(tick1);
}
static void tick2_callback(const std::string& channel) {
  process_ticks(tick2);
}
static void tick3_callback(const std::string& channel) {
  process_ticks(tick3);
}
static void tick4_callback(const std::string& channel) {
  process_ticks(tick4);
}

class OdometryNode : public rclcpp::Node {
  public:
      OdometryNode() : Node("odometry_node")
      {
        transform_publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        timer_ = this->create_wall_timer(
        1000ms, std::bind(&OdometryNode::timer_callback, this));

        tickers_.push_back(tick1);
        tickers_.push_back(tick2);
        tickers_.push_back(tick3);
        tickers_.push_back(tick4);

        GPIO::add_event_detect(tick1->enca_, GPIO::BOTH, tick1_callback, 0);
        GPIO::add_event_detect(tick2->enca_, GPIO::BOTH, tick2_callback, 0);
        GPIO::add_event_detect(tick3->enca_, GPIO::BOTH, tick3_callback, 0);
        GPIO::add_event_detect(tick4->enca_, GPIO::BOTH, tick4_callback, 0);

        odom_data_.linear_x = 0.0;
        odom_data_.linear_y = 0.0;
        odom_data_.angular_z = 0.0;
        odom_data_.pose_x = 0.0;
        odom_data_.pose_y = 0.0;
        odom_data_.orientation_z = 0.0;

        start_time_ = std::chrono::high_resolution_clock::now();

        RCLCPP_INFO(this->get_logger(), "Odometry Node Started");
      }

  void calc_odometry(odom_data_t & odom_data, float elapsed_time) {

    auto v0 = tickers_.at(0)->average_velocity;
    auto v1 = tickers_.at(1)->average_velocity;
    auto v2 = tickers_.at(2)->average_velocity;
    auto v3 = tickers_.at(3)->average_velocity;

    odom_data.linear_x = (v0 + v1 + v2 + v3 ) / 4;
    odom_data.linear_y = (-v0 + v1 + v2 - v3) / 4;
    odom_data.angular_z = -((v0 - v1 + v2 - v3) * WHEEL_RADIUS / (2 * (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y)));

    float dx = odom_data.linear_x * elapsed_time;
    float dy = odom_data.linear_y * elapsed_time;
    float dxy = sqrt(dx*dx + dy*dy);
    float dtheta = odom_data.angular_z * elapsed_time;

    odom_data.pose_x += cos(odom_data.orientation_z + dtheta/2)*dxy; // + sin(odom_data.orientation_z)*dy;
    odom_data.pose_y += sin(odom_data.orientation_z + dtheta/2)*dxy; // + sin(odom_data.orientation_z)*dx;
    //odom_data.orientation_z += dtheta;
    int degrees = odom_data.orientation_z = std::lround((odom_data.orientation_z + dtheta) / RADIANS) % 360;
    odom_data.orientation_z = (float)degrees * RADIANS;
    

}

    void publish_odometry(float elapsed_time) {
      Odometry odom;
      TransformStamped tf;

      auto stamp = this->get_clock()->now();

      double pcov[36] = { 0.1,   0,   0,   0,   0, 0,
                        0, 0.1,   0,   0,   0, 0,
                        0,   0, 1e6,   0,   0, 0,
                        0,   0,   0, 1e6,   0, 0,
                        0,   0,   0,   0, 1e6, 0,
                        0,   0,   0,   0,   0, 0.2};

      memcpy(&(odom.pose.covariance),pcov,sizeof(double)*36);
      memcpy(&(odom.twist.covariance),pcov,sizeof(double)*36);

      calc_odometry(odom_data_, elapsed_time);

      odom.header.stamp = stamp;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = odom_data_.linear_x;
      odom.twist.twist.linear.y = odom_data_.linear_y;
      odom.twist.twist.angular.z = odom_data_.angular_z;
      odom.pose.pose.position.x = odom_data_.pose_x;
      odom.pose.pose.position.y = odom_data_.pose_y;
      tf2::Quaternion q;
      q.setRPY(0, 0, odom_data_.orientation_z);
      odom.pose.pose.orientation.x = q.x();
      odom.pose.pose.orientation.y = q.y();
      odom.pose.pose.orientation.z = q.z();
      odom.pose.pose.orientation.w = q.w();

      RCLCPP_INFO(this->get_logger(), "lX: %f lY: %f aZ: %f", odom_data_.linear_x, odom_data_.linear_y, odom_data_.angular_z);

      RCLCPP_INFO(this->get_logger(), "X: %f Y: %f Z: %f W: %f", odom_data_.pose_x, odom_data_.pose_y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);

      tf.header.stamp = stamp;
      tf.header.frame_id = "odom";
      tf.child_frame_id = "base_link";
      tf.transform.translation.x = odom_data_.pose_x;
      tf.transform.translation.y = odom_data_.pose_y;
      tf.transform.rotation.x = q.x();
      tf.transform.rotation.x = q.y();
      tf.transform.rotation.x = q.z();
      tf.transform.rotation.x = q.w();
    
      TFMessage transforms;
      transforms.transforms.push_back(tf);

      odom_publisher_->publish(odom);
      transform_publisher_->publish(transforms);

    }
    void timer_callback()
    {

      auto end_time = system_clock::now();
      float elapsed_time = (float) duration_cast<microseconds>(end_time - start_time_).count() / 1000000.0;
      
      for(int i=0; i < 4; i++) {
        int tick_difference = tickers_.at(i)->ticks - tickers_.at(i)->start_ticks;
        float rpm = (((float)tick_difference / (float)TICKS_PER_REVOLUTION) / (elapsed_time / 60.0));
        tickers_.at(i)->set_rpm(rpm);
        tickers_.at(i)->start_ticks = tickers_.at(i)->ticks;
        start_time_ = std::chrono::high_resolution_clock::now();

        //RCLCPP_INFO(this->get_logger(), "RPM: %d: %f VEL: %f Ticks: %d Duration %f", i, tickers_.at(i)->average_rpm, tickers_.at(i)->average_velocity, tick_difference, elapsed_time);
      }

      publish_odometry(elapsed_time);

      

    }

  private:
    std::chrono::_V2::system_clock::time_point start_time_;
    odom_data_t odom_data_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr transform_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::vector<TicksCounter*> tickers_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryNode>());
  rclcpp::shutdown();
  return 0;
}
