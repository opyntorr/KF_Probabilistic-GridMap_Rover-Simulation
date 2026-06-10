#include <chrono>
#include <sstream>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Inertial.hh>
#include <ignition/gazebo/components/ExternalWorldWrenchCmd.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/msgs/Utility.hh>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tello_msgs/msg/flight_data.hpp"
#include "tello_msgs/msg/tello_response.hpp"
#include "tello_msgs/srv/tello_action.hpp"

#include "pid.hpp"

using namespace std::chrono_literals;

namespace tello_gazebo
{
  const double MAX_XY_V = 8.0;
  const double MAX_Z_V = 4.0;
  const double MAX_ANG_V = M_PI;

  const double MAX_XY_A = 8.0;
  const double MAX_Z_A = 4.0;
  const double MAX_ANG_A = M_PI;

  const double TAKEOFF_Z = 1.0;
  const double TAKEOFF_Z_V = 0.5;

  const double LAND_Z = 0.1;
  const double LAND_Z_V = -0.5;

  const int BATTERY_DURATION = 6000;

  inline double clamp(const double v, const double max)
  {
    return v > max ? max : (v < -max ? -max : v);
  }

  class TelloPlugin : public ::ignition::gazebo::System,
                      public ::ignition::gazebo::ISystemConfigure,
                      public ::ignition::gazebo::ISystemPreUpdate,
                      public ::ignition::gazebo::ISystemPostUpdate
  {
    enum class FlightState
    {
      landed,
      taking_off,
      flying,
      landing,
      dead_battery,
    };

    std::map<FlightState, const char *> state_strs_{
      {FlightState::landed,       "landed"},
      {FlightState::taking_off,   "taking_off"},
      {FlightState::flying,       "flying"},
      {FlightState::landing,      "landing"},
      {FlightState::dead_battery, "dead_battery"},
    };

    FlightState flight_state_{FlightState::landed};

    ::ignition::gazebo::Entity model_entity_;
    ::ignition::gazebo::Link base_link_;
    ::ignition::math::Vector3d gravity_{0, 0, -9.81};
    int battery_duration_{BATTERY_DURATION};

    // ROS 2
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<tello_msgs::msg::FlightData>::SharedPtr flight_data_pub_;
    rclcpp::Publisher<tello_msgs::msg::TelloResponse>::SharedPtr tello_response_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Service<tello_msgs::srv::TelloAction>::SharedPtr command_srv_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    // Sim time
    std::chrono::steady_clock::duration last_update_time_{0};
    std::chrono::steady_clock::duration last_10hz_time_{0};

    // PID Controllers
    pid::Controller x_controller_{false, 2, 0, 0};
    pid::Controller y_controller_{false, 2, 0, 0};
    pid::Controller z_controller_{false, 2, 0, 0};
    pid::Controller yaw_controller_{false, 2, 0, 0};

    // Targets
    double target_x_v_{0};
    double target_y_v_{0};
    double target_z_v_{0};
    double target_yaw_v_{0};

    // Spawn height (set on first spin_10Hz call to compute relative takeoff/land thresholds)
    double spawn_z_{0.0};
    bool spawn_z_set_{false};

  public:
    TelloPlugin() = default;
    ~TelloPlugin() = default;

    void Configure(const ::ignition::gazebo::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   ::ignition::gazebo::EntityComponentManager &_ecm,
                   ::ignition::gazebo::EventManager &/*_eventMgr*/) override
    {
      model_entity_ = _entity;
      ::ignition::gazebo::Model model(_entity);
      std::string link_name = _sdf->Get<std::string>("link_name", "base_link").first;
      base_link_ = ::ignition::gazebo::Link(model.LinkByName(_ecm, link_name));

      if (!base_link_.Valid(_ecm)) {
        ignerr << "TelloPlugin: Link [" << link_name << "] not found!" << std::endl;
        return;
      }

      // Enable velocity components so we can read them in PreUpdate
      base_link_.EnableVelocityChecks(_ecm, true);

      if (_sdf->HasElement("battery_duration")) {
        battery_duration_ = _sdf->Get<int>("battery_duration");
      }

      // ROS 2 Initialization
      if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
      }

      std::string ns = _sdf->Get<std::string>("namespace", "drone1").first;
      node_ = std::make_shared<rclcpp::Node>("tello_plugin_node", ns);

      flight_data_pub_ = node_->create_publisher<tello_msgs::msg::FlightData>("flight_data", 1);
      tello_response_pub_ = node_->create_publisher<tello_msgs::msg::TelloResponse>("tello_response", 1);
      odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
      command_srv_ = node_->create_service<tello_msgs::srv::TelloAction>("tello_action",
        std::bind(&TelloPlugin::command_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
      cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
        std::bind(&TelloPlugin::cmd_vel_callback, this, std::placeholders::_1));

      ignmsg << "TelloPlugin loaded for model [" << model.Name(_ecm) << "] in namespace [" << ns << "]" << std::endl;
    }

    void PreUpdate(const ::ignition::gazebo::UpdateInfo &_info,
                   ::ignition::gazebo::EntityComponentManager &_ecm) override
    {
      if (_info.paused) return;

      if (flight_state_ == FlightState::dead_battery) return;

      double dt = std::chrono::duration<double>(_info.simTime - last_update_time_).count();
      if (last_update_time_.count() == 0) dt = 0.001;
      last_update_time_ = _info.simTime;

      if (flight_state_ != FlightState::landed) {
        auto lin_vel = base_link_.WorldLinearVelocity(_ecm);
        auto ang_vel = base_link_.WorldAngularVelocity(_ecm);
        auto world_pose = ::ignition::gazebo::worldPose(base_link_.Entity(), _ecm);

        if (!lin_vel || !ang_vel) return;

        // Convert world velocity to body velocity
        ::ignition::math::Vector3d body_lin_vel = world_pose.Rot().Inverse().RotateVector(*lin_vel);
        ::ignition::math::Vector3d body_ang_vel = world_pose.Rot().Inverse().RotateVector(*ang_vel);

        // PID calculations
        ::ignition::math::Vector3d lin_ubar, ang_ubar;

        lin_ubar.X(x_controller_.calc(body_lin_vel.X(), dt, 0));
        lin_ubar.Y(y_controller_.calc(body_lin_vel.Y(), dt, 0));
        lin_ubar.Z(z_controller_.calc(body_lin_vel.Z(), dt, 0));
        ang_ubar.Z(yaw_controller_.calc(body_ang_vel.Z(), dt, 0));

        lin_ubar.X() = clamp(lin_ubar.X(), MAX_XY_A);
        lin_ubar.Y() = clamp(lin_ubar.Y(), MAX_XY_A);
        lin_ubar.Z() = clamp(lin_ubar.Z(), MAX_Z_A);
        ang_ubar.Z() = clamp(ang_ubar.Z(), MAX_ANG_A);

        // Gravity compensation (in body frame)
        ::ignition::math::Vector3d body_gravity = world_pose.Rot().Inverse().RotateVector(gravity_);
        lin_ubar -= body_gravity;

        // Get mass from Inertial component
        auto inertial_comp = _ecm.Component<::ignition::gazebo::components::Inertial>(
            base_link_.Entity());
        if (!inertial_comp) return;

        double mass = inertial_comp->Data().MassMatrix().Mass();

        // Extract diagonal of MOI manually (Matrix3 has no Diagonal() method)
        ::ignition::math::Matrix3d moi = inertial_comp->Data().MassMatrix().Moi();
        ::ignition::math::Vector3d moi_diag(moi(0, 0), moi(1, 1), moi(2, 2));

        ::ignition::math::Vector3d force = lin_ubar * mass;
        ::ignition::math::Vector3d torque(
            ang_ubar.X() * moi_diag.X(),
            ang_ubar.Y() * moi_diag.Y(),
            ang_ubar.Z() * moi_diag.Z());

        // Transform to world frame
        ::ignition::math::Vector3d world_force = world_pose.Rot().RotateVector(force);
        ::ignition::math::Vector3d world_torque = world_pose.Rot().RotateVector(torque);

        // Construct the Wrench protobuf message manually
        ::ignition::msgs::Wrench wrenchMsg;
        ::ignition::msgs::Set(wrenchMsg.mutable_force(), world_force);
        ::ignition::msgs::Set(wrenchMsg.mutable_torque(), world_torque);

        // Apply wrench
        auto wrenchComp = _ecm.Component<::ignition::gazebo::components::ExternalWorldWrenchCmd>(
            base_link_.Entity());
        if (wrenchComp) {
          *wrenchComp = ::ignition::gazebo::components::ExternalWorldWrenchCmd(wrenchMsg);
        } else {
          _ecm.CreateComponent(base_link_.Entity(),
              ::ignition::gazebo::components::ExternalWorldWrenchCmd(wrenchMsg));
        }
      }
    }

    void PostUpdate(const ::ignition::gazebo::UpdateInfo &_info,
                    const ::ignition::gazebo::EntityComponentManager &_ecm) override
    {
      if (_info.paused) return;

      // Process ROS callbacks synchronously (replaces the separate spin thread)
      if (rclcpp::ok()) {
        rclcpp::spin_some(node_);
      }

      if ((_info.simTime - last_10hz_time_) >= 100ms) {
        spin_10Hz(_info, _ecm);
        last_10hz_time_ = _info.simTime;
      }
    }

    void spin_10Hz(const ::ignition::gazebo::UpdateInfo &_info,
                   const ::ignition::gazebo::EntityComponentManager &_ecm)
    {
      double sim_time_sec = std::chrono::duration<double>(_info.simTime).count();
      if (sim_time_sec < 0.1) return;

      auto world_pose_now = ::ignition::gazebo::worldPose(base_link_.Entity(), _ecm);
      if (!spawn_z_set_) {
        spawn_z_ = world_pose_now.Pos().Z();
        spawn_z_set_ = true;
        RCLCPP_INFO(node_->get_logger(), "Spawn Z recorded: %.3f m", spawn_z_);
      }

      int battery_percent = static_cast<int>((battery_duration_ - sim_time_sec) / battery_duration_ * 100);
      if (battery_percent <= 0) {
        transition(FlightState::dead_battery);
        return;
      }

      tello_msgs::msg::FlightData flight_data;
      flight_data.header.stamp = node_->now();
      flight_data.sdk = flight_data.SDK_1_3;
      flight_data.bat = battery_percent;
      flight_data_pub_->publish(flight_data);

      auto world_pose = world_pose_now;

      // Publish ground truth odometry directly on ROS (replaces gazebo_ros_p3d)
      auto lin_vel = base_link_.WorldLinearVelocity(_ecm);
      auto ang_vel = base_link_.WorldAngularVelocity(_ecm);

      nav_msgs::msg::Odometry odom_msg;
      odom_msg.header.stamp = node_->now();
      odom_msg.header.frame_id = "map";
      odom_msg.child_frame_id = "base_link";

      odom_msg.pose.pose.position.x = world_pose.Pos().X();
      odom_msg.pose.pose.position.y = world_pose.Pos().Y();
      odom_msg.pose.pose.position.z = world_pose.Pos().Z();
      odom_msg.pose.pose.orientation.x = world_pose.Rot().X();
      odom_msg.pose.pose.orientation.y = world_pose.Rot().Y();
      odom_msg.pose.pose.orientation.z = world_pose.Rot().Z();
      odom_msg.pose.pose.orientation.w = world_pose.Rot().W();

      if (lin_vel) {
        odom_msg.twist.twist.linear.x = lin_vel->X();
        odom_msg.twist.twist.linear.y = lin_vel->Y();
        odom_msg.twist.twist.linear.z = lin_vel->Z();
      }
      if (ang_vel) {
        odom_msg.twist.twist.angular.x = ang_vel->X();
        odom_msg.twist.twist.angular.y = ang_vel->Y();
        odom_msg.twist.twist.angular.z = ang_vel->Z();
      }

      odom_pub_->publish(odom_msg);

      if (flight_state_ == FlightState::taking_off && world_pose.Pos().Z() > spawn_z_ + TAKEOFF_Z) {
        transition(FlightState::flying);
        respond_ok();
      } else if (flight_state_ == FlightState::landing && world_pose.Pos().Z() < spawn_z_ + LAND_Z) {
        transition(FlightState::landed);
        respond_ok();
      }
    }

    void transition(FlightState next)
    {
      RCLCPP_INFO(node_->get_logger(), "Transition from '%s' to '%s'",
                  state_strs_[flight_state_], state_strs_[next]);
      flight_state_ = next;

      switch (flight_state_) {
        case FlightState::landed:
        case FlightState::flying:
        case FlightState::dead_battery:
          x_controller_.set_target(0);
          y_controller_.set_target(0);
          z_controller_.set_target(0);
          yaw_controller_.set_target(0);
          target_x_v_ = target_y_v_ = target_yaw_v_ = 0;
          target_z_v_ = 0;
          break;
        case FlightState::taking_off:
          x_controller_.set_target(0);
          y_controller_.set_target(0);
          z_controller_.set_target(TAKEOFF_Z_V);
          yaw_controller_.set_target(0);
          target_x_v_ = target_y_v_ = target_yaw_v_ = 0;
          target_z_v_ = TAKEOFF_Z_V;
          break;
        case FlightState::landing:
          x_controller_.set_target(0);
          y_controller_.set_target(0);
          z_controller_.set_target(LAND_Z_V);
          yaw_controller_.set_target(0);
          target_x_v_ = target_y_v_ = target_yaw_v_ = 0;
          target_z_v_ = LAND_Z_V;
          break;
      }
    }

    void respond_ok()
    {
      tello_msgs::msg::TelloResponse msg;
      msg.rc = msg.OK;
      msg.str = "ok";
      tello_response_pub_->publish(msg);
    }

    void command_callback(const std::shared_ptr<rmw_request_id_t>,
                         const std::shared_ptr<tello_msgs::srv::TelloAction::Request> request,
                         std::shared_ptr<tello_msgs::srv::TelloAction::Response> response)
    {
      if (request->cmd == "takeoff" && flight_state_ == FlightState::landed) {
        transition(FlightState::taking_off);
        response->rc = response->OK;
      } else if (request->cmd == "land" && flight_state_ == FlightState::flying) {
        transition(FlightState::landing);
        response->rc = response->OK;
      } else if (request->cmd.substr(0, 2) == "rc" && flight_state_ == FlightState::flying) {
        parse_rc_command(request->cmd);
        response->rc = response->OK;
      } else {
        RCLCPP_WARN(node_->get_logger(), "Ignoring command '%s'", request->cmd.c_str());
        response->rc = response->ERROR_BUSY;
      }
    }

    void parse_rc_command(const std::string &rc_command)
    {
      double x, y, z, yaw;
      try {
        std::istringstream iss(rc_command);
        std::string s;
        iss >> s >> x >> y >> z >> yaw;
        target_x_v_ = x * MAX_XY_V;
        target_y_v_ = y * MAX_XY_V;
        target_z_v_ = z * MAX_Z_V;
        target_yaw_v_ = yaw * MAX_ANG_V;
        x_controller_.set_target(target_x_v_);
        y_controller_.set_target(target_y_v_);
        z_controller_.set_target(target_z_v_);
        yaw_controller_.set_target(target_yaw_v_);
      } catch (...) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to parse RC command: %s", rc_command.c_str());
      }
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      if (flight_state_ == FlightState::flying) {
        target_x_v_ = msg->linear.x * MAX_XY_V;
        target_y_v_ = msg->linear.y * MAX_XY_V;
        target_z_v_ = msg->linear.z * MAX_Z_V;
        target_yaw_v_ = msg->angular.z * MAX_ANG_V;
        x_controller_.set_target(target_x_v_);
        y_controller_.set_target(target_y_v_);
        z_controller_.set_target(target_z_v_);
        yaw_controller_.set_target(target_yaw_v_);
      }
    }
  };

} // namespace tello_gazebo

// Register plugin OUTSIDE the namespace to avoid name resolution issues
IGNITION_ADD_PLUGIN(tello_gazebo::TelloPlugin,
                    ::ignition::gazebo::System,
                    tello_gazebo::TelloPlugin::ISystemConfigure,
                    tello_gazebo::TelloPlugin::ISystemPreUpdate,
                    tello_gazebo::TelloPlugin::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(tello_gazebo::TelloPlugin, "tello_gazebo::TelloPlugin")
