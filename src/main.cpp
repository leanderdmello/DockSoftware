#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/uint8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>

using namespace std::chrono_literals;

enum DockStatus : uint8_t { FREE=0, RESERVED=1, OCCUPIED=2 };
enum DroneLandingState : uint8_t { D_IDLE=0, D_LANDING=1, D_LANDED=2 };
enum DroneSimpleState  : uint8_t { U_UNKNOWN=0, U_FLYING=1, U_DISARMED=2 };

class DockNode : public rclcpp::Node {
public:
  DockNode() : rclcpp::Node("dock_comm"), status_(FREE), last_landing_state_(D_IDLE) {
    reserve_timeout_s_ = declare_parameter<int>("reserve_timeout_s", 60);
    occupied_hold_s_   = declare_parameter<int>("occupied_hold_s", 10);
    qos_depth_         = declare_parameter<int>("qos_depth", 10);

    status_pub_ = create_publisher<std_msgs::msg::UInt8>("/dock/status", rclcpp::QoS(qos_depth_));
    ctl_pub_    = create_publisher<std_msgs::msg::Bool> ("/drone/landing/clear_to_land", rclcpp::QoS(qos_depth_));

    req_sub_ = create_subscription<std_msgs::msg::Empty>("/dock/landing/request", rclcpp::QoS(qos_depth_),
      [this](std_msgs::msg::Empty::SharedPtr){ on_request(); });

    landing_state_sub_ = create_subscription<std_msgs::msg::UInt8>("/drone/landing/state", rclcpp::QoS(qos_depth_),
      [this](std_msgs::msg::UInt8::SharedPtr m){ on_landing_state(m->data); });

    drone_simple_sub_ = create_subscription<std_msgs::msg::UInt8>("/drone/state_simple", rclcpp::QoS(qos_depth_),
      [this](std_msgs::msg::UInt8::SharedPtr m){ on_drone_simple(m->data); });

    publish_status(FREE);
    RCLCPP_INFO(get_logger(), "Dock node up. reserve_timeout_s=%d, occupied_hold_s=%d",
                reserve_timeout_s_, occupied_hold_s_);
    RCLCPP_INFO(get_logger(), "Waiting for /dock/landing/request from drone_sim...");
  }

private:
  void on_request() {
    RCLCPP_INFO(get_logger(), "[Dock] Received: requestLanding");
    if (status_ == FREE) {
      RCLCPP_INFO(get_logger(), "[Dock] Status FREE -> APPROVE (clear_to_land=true), set RESERVED, start timer");
      send_clear_to_land(true);
      set_reserved_with_timer();
    } else {
      RCLCPP_WARN(get_logger(), "[Dock] Not FREE (status=%u) -> DENY (clear_to_land=false)", status_);
      send_clear_to_land(false);
    }
  }

  void on_landing_state(uint8_t s) {
    last_landing_state_ = s;
    if (s == D_LANDING) {
      RCLCPP_INFO(get_logger(), "[Dock] Drone state: LANDING -> keep RESERVED, refresh timer");
      if (status_ != RESERVED) {
        set_reserved_with_timer();
      } else {
        refresh_reserve_timer();
      }
    } else if (s == D_LANDED) {
      RCLCPP_INFO(get_logger(), "[Dock] Drone state: LANDED -> set OCCUPIED, stop timer, hold %ds", occupied_hold_s_);
      cancel_reserve_timer();
      set_status(OCCUPIED);
      start_occupied_hold_timer();
    } else {
      RCLCPP_INFO(get_logger(), "[Dock] Drone state: IDLE");
    }
  }

  void on_drone_simple(uint8_t s) {
    if (s == U_FLYING && last_landing_state_ != D_LANDING) {
      if (status_ != FREE) {
        RCLCPP_INFO(get_logger(), "[Dock] Drone is FLYING and not landing -> set FREE");
        cancel_reserve_timer();
        cancel_occupied_timer();
        set_status(FREE);
      }
    }
  }

  void set_reserved_with_timer() { set_status(RESERVED); refresh_reserve_timer(); }

  void refresh_reserve_timer() {
    reserve_timer_ = create_wall_timer(std::chrono::seconds(reserve_timeout_s_), [this]() {
      RCLCPP_WARN(get_logger(), "[Dock] Reservation timed out -> set FREE, send clear_to_land=false");
      set_status(FREE);
      send_clear_to_land(false);
      reserve_timer_.reset();
    });
  }

  void start_occupied_hold_timer() {
    cancel_occupied_timer();
    occupied_timer_ = create_wall_timer(std::chrono::seconds(occupied_hold_s_), [this]() {
      RCLCPP_INFO(get_logger(), "[Dock] OCCUPIED hold finished (still OCCUPIED until cleared externally)");
      occupied_timer_.reset();
    });
  }

  void cancel_reserve_timer()  { if (reserve_timer_)  reserve_timer_.reset(); }
  void cancel_occupied_timer() { if (occupied_timer_) occupied_timer_.reset(); }

  void set_status(uint8_t s) { status_ = s; publish_status(s); }
  void publish_status(uint8_t s) { std_msgs::msg::UInt8 m; m.data = s; status_pub_->publish(m); }

  void send_clear_to_land(bool ok) { std_msgs::msg::Bool m; m.data = ok; ctl_pub_->publish(m); }

  int reserve_timeout_s_{60}, occupied_hold_s_{10}, qos_depth_{10};
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr  ctl_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr  req_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr  landing_state_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr  drone_simple_sub_;
  rclcpp::TimerBase::SharedPtr reserve_timer_, occupied_timer_;
  uint8_t status_, last_landing_state_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DockNode>());
  rclcpp::shutdown();
  return 0;
}
