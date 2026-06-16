#include <cmath>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class ColorGraspMoveIt : public rclcpp::Node
{
public:
  explicit ColorGraspMoveIt(const rclcpp::NodeOptions & options)
  : Node("color_grasp_moveit", options)
  {
    target_pose_topic_ = declare_parameter<std::string>("target_pose_topic", "/vision/target_pose");
    execute_on_target_ = declare_parameter<bool>("execute_on_target", false);
    approach_height_m_ = declare_parameter<double>("approach_height_m", 0.08);
    grasp_height_m_ = declare_parameter<double>("grasp_height_m", 0.02);
    lift_height_m_ = declare_parameter<double>("lift_height_m", 0.20);
    lift_wait_sec_ = declare_parameter<double>("lift_wait_sec", 1.0);
    use_joint_lift_pose_ = declare_parameter<bool>("use_joint_lift_pose", true);
    lift_joint1_rad_ = declare_parameter<double>("lift_joint1_rad", -1.4835298642);
    lift_joint2_rad_ = declare_parameter<double>("lift_joint2_rad", -0.1919862177);
    lift_joint3_rad_ = declare_parameter<double>("lift_joint3_rad", -0.4712388980);
    lift_joint4_rad_ = declare_parameter<double>("lift_joint4_rad", 2.181661565);
    move_to_start_pose_on_startup_ = declare_parameter<bool>("move_to_start_pose_on_startup", true);
    startup_start_pose_delay_sec_ = declare_parameter<double>("startup_start_pose_delay_sec", 3.0);
    pre_motor_pose1_joint1_rad_ = declare_parameter<double>("pre_motor_pose1_joint1_rad", -1.5358897418);
    pre_motor_pose1_joint2_rad_ = declare_parameter<double>("pre_motor_pose1_joint2_rad", 0.3316125579);
    pre_motor_pose1_joint3_rad_ = declare_parameter<double>("pre_motor_pose1_joint3_rad", 0.1919862177);
    pre_motor_pose1_joint4_rad_ = declare_parameter<double>("pre_motor_pose1_joint4_rad", 0.4363323130);
    pre_motor_pose2_joint1_rad_ = declare_parameter<double>("pre_motor_pose2_joint1_rad", -1.5358897418);
    pre_motor_pose2_joint2_rad_ = declare_parameter<double>("pre_motor_pose2_joint2_rad", 1.0297442587);
    pre_motor_pose2_joint3_rad_ = declare_parameter<double>("pre_motor_pose2_joint3_rad", 0.2268928028);
    pre_motor_pose2_joint4_rad_ = declare_parameter<double>("pre_motor_pose2_joint4_rad", 0.1570796327);
    drop_pose1_joint1_rad_ = declare_parameter<double>("drop_pose1_joint1_rad", -2.3911010752);
    drop_pose1_joint2_rad_ = declare_parameter<double>("drop_pose1_joint2_rad", -0.2617993878);
    drop_pose1_joint3_rad_ = declare_parameter<double>("drop_pose1_joint3_rad", 0.0);
    drop_pose1_joint4_rad_ = declare_parameter<double>("drop_pose1_joint4_rad", 1.6929693744);
    drop_pose2_joint1_rad_ = declare_parameter<double>("drop_pose2_joint1_rad", -2.3911010752);
    drop_pose2_joint2_rad_ = declare_parameter<double>("drop_pose2_joint2_rad", 0.5410520681);
    drop_pose2_joint3_rad_ = declare_parameter<double>("drop_pose2_joint3_rad", -0.6457718232);
    drop_pose2_joint4_rad_ = declare_parameter<double>("drop_pose2_joint4_rad", 1.6929693744);
    drop_pose3_joint1_rad_ = declare_parameter<double>("drop_pose3_joint1_rad", -2.3911010752);
    drop_pose3_joint2_rad_ = declare_parameter<double>("drop_pose3_joint2_rad", 0.9599310886);
    drop_pose3_joint3_rad_ = declare_parameter<double>("drop_pose3_joint3_rad", -0.5934119457);
    drop_pose3_joint4_rad_ = declare_parameter<double>("drop_pose3_joint4_rad", 1.2566370614);
    drop_pose4_joint1_rad_ = declare_parameter<double>("drop_pose4_joint1_rad", -2.3911010752);
    drop_pose4_joint2_rad_ = declare_parameter<double>("drop_pose4_joint2_rad", 1.4311699866);
    drop_pose4_joint3_rad_ = declare_parameter<double>("drop_pose4_joint3_rad", -0.7679448709);
    drop_pose4_joint4_rad_ = declare_parameter<double>("drop_pose4_joint4_rad", 0.7504915784);
    cartesian_eef_step_m_ = declare_parameter<double>("cartesian_eef_step_m", 0.01);
    cartesian_jump_threshold_ = declare_parameter<double>("cartesian_jump_threshold", 0.0);
    cartesian_min_fraction_ = declare_parameter<double>("cartesian_min_fraction", 0.9);
    use_cartesian_grasp_descent_ = declare_parameter<bool>("use_cartesian_grasp_descent", true);
    lock_joint4_during_grasp_descent_ = declare_parameter<bool>(
      "lock_joint4_during_grasp_descent", true);
    grasp_joint4_lock_tolerance_rad_ = declare_parameter<double>(
      "grasp_joint4_lock_tolerance_rad", 0.08);
    assume_close_after_timeout_ = declare_parameter<bool>("assume_close_after_timeout", true);
    close_gripper_timeout_sec_ = declare_parameter<double>("close_gripper_timeout_sec", 1.5);
    close_gripper_on_grasp_motion_failure_ = declare_parameter<bool>(
      "close_gripper_on_grasp_motion_failure", true);
    close_gripper_before_pre_motor_pose_ = declare_parameter<bool>(
      "close_gripper_before_pre_motor_pose", true);
    use_custom_close_position_ = declare_parameter<bool>("use_custom_close_position", true);
    close_gripper_joint_name_ = declare_parameter<std::string>(
      "close_gripper_joint_name", "gripper_left_joint");
    close_gripper_joint_position_m_ = declare_parameter<double>(
      "close_gripper_joint_position_m", -0.0010);
    auto_execute_cooldown_sec_ = declare_parameter<double>("auto_execute_cooldown_sec", 10.0);
    max_target_radius_m_ = declare_parameter<double>("max_target_radius_m", 0.36);
    last_auto_execute_time_ = now() - rclcpp::Duration::from_seconds(auto_execute_cooldown_sec_);

    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      target_pose_topic_, 10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(target_mutex_);
        latest_target_ = *msg;
        have_target_ = true;
        if (execute_on_target_ && !executing_ && autoCooldownElapsed()) {
          pending_auto_execute_ = true;
        }
      });

    pick_service_ = create_service<std_srvs::srv::Trigger>(
      "pick_latest_target",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = pickLatestTarget();
        response->message = response->success ? "pick sequence finished" : "pick sequence failed";
      });

    finish_service_ = create_service<std_srvs::srv::Trigger>(
      "finish_grasp_pose",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = finishGraspPose();
        response->message = response->success ? "finish grasp pose finished" : "finish grasp pose failed";
      });

    return_service_ = create_service<std_srvs::srv::Trigger>(
      "return_to_start_pose",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = moveArmToJointLiftPose();
        response->message = response->success ? "return to start pose succeeded" : "return to start pose failed";
      });

    pre_motor_pose_service_ = create_service<std_srvs::srv::Trigger>(
      "move_pre_motor_pose_sequence",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = moveArmToPreMotorPoseSequence();
        response->message = response->success ?
          "pre motor pose sequence succeeded" : "pre motor pose sequence failed";
      });

    drop_service_ = create_service<std_srvs::srv::Trigger>(
      "drop_mine_at_safe_zone",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = dropMineAtSafeZone();
        response->message = response->success ?
          "drop mine at safe zone succeeded" : "drop mine at safe zone failed";
      });

    capture_service_ = create_service<std_srvs::srv::Trigger>(
      "capture_pending_target",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        std::lock_guard<std::mutex> lock(target_mutex_);
        if (!have_target_) {
          response->success = false;
          response->message = "no latest target to capture";
          RCLCPP_WARN(get_logger(), "capture_pending_target failed: no latest target.");
          return;
        }
        pending_target_ = latest_target_;
        have_pending_target_ = true;
        response->success = true;
        response->message = "pending target captured";
        RCLCPP_INFO(get_logger(), "capture_pending_target: captured latest target.");
      });

    auto_timer_ = create_wall_timer(
      std::chrono::milliseconds(300),
      [this]() {
        if (!pending_auto_execute_) {
          return;
        }
        pending_auto_execute_ = false;
        if (executing_ || !autoCooldownElapsed()) {
          return;
        }
        executing_ = true;
        const bool ok = pickLatestTarget();
        last_auto_execute_time_ = now();
        executing_ = false;
        if (!ok) {
          RCLCPP_WARN(get_logger(), "Automatic pick attempt failed.");
        }
      });
  }

  void initMoveGroups()
  {
    const auto arm_group = get_parameter_or<std::string>("arm_group", "arm");
    const auto gripper_group = get_parameter_or<std::string>("gripper_group", "gripper");
    arm_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), arm_group);
    gripper_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), gripper_group);
    arm_->setPlanningTime(5.0);
    arm_->setGoalPositionTolerance(get_parameter_or<double>("position_tolerance_m", 0.02));
    arm_->setGoalOrientationTolerance(get_parameter_or<double>("orientation_tolerance_rad", 0.25));

    if (move_to_start_pose_on_startup_) {
      double delay_sec = startup_start_pose_delay_sec_;
      if (delay_sec < 0.0) {
        delay_sec = 0.0;
      }
      startup_pose_timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(delay_sec * 1000.0)),
        [this]() {
          if (startup_pose_timer_) {
            startup_pose_timer_->cancel();
          }
          if (executing_) {
            RCLCPP_WARN(get_logger(), "Skipping startup start pose move because another motion is running.");
            return;
          }
          executing_ = true;
          RCLCPP_INFO(get_logger(), "Moving arm to start pose on startup.");
          const bool ok = moveArmToJointLiftPose();
          executing_ = false;
          if (!ok) {
            RCLCPP_WARN(get_logger(), "Startup start pose move failed.");
          }
        });
    }
  }

private:
  bool pickLatestTarget()
  {
    geometry_msgs::msg::PoseStamped target;
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      if (!have_target_) {
        RCLCPP_WARN(get_logger(), "pickLatestTarget failed: no target pose received yet.");
        return false;
      }
      target = latest_target_;
    }

    RCLCPP_INFO(
      get_logger(),
      "pickLatestTarget: target pose [x=%.3f y=%.3f z=%.3f]",
      target.pose.position.x,
      target.pose.position.y,
      target.pose.position.z);

    if (!isTargetInsideWorkspace(target)) {
      RCLCPP_WARN(
        get_logger(),
        "pickLatestTarget rejected target outside workspace: x=%.3f y=%.3f z=%.3f",
        target.pose.position.x, target.pose.position.y, target.pose.position.z);
      return false;
    }

    const auto open_target = get_parameter_or<std::string>("open_gripper_target", "open");
    const auto close_target = get_parameter_or<std::string>("close_gripper_target", "close");

    geometry_msgs::msg::PoseStamped pregrasp = target;
    pregrasp.pose.position.z += approach_height_m_;
    geometry_msgs::msg::PoseStamped grasp = target;
    grasp.pose.position.z += grasp_height_m_;

    if (!moveGripper(open_target)) {
      RCLCPP_WARN(get_logger(), "pickLatestTarget failed: opening gripper failed.");
      return false;
    }
    if (!moveArm(pregrasp, "pregrasp")) {
      RCLCPP_WARN(get_logger(), "pickLatestTarget failed: pregrasp motion failed.");
      return false;
    }
    const bool grasp_motion_succeeded = moveToGrasp(grasp, "grasp");
    if (!grasp_motion_succeeded && !close_gripper_on_grasp_motion_failure_) {
      RCLCPP_WARN(get_logger(), "pickLatestTarget failed: grasp motion failed.");
      return false;
    }
    if (!grasp_motion_succeeded) {
      RCLCPP_WARN(
        get_logger(),
        "pickLatestTarget: grasp motion failed, but closing gripper anyway because "
        "close_gripper_on_grasp_motion_failure is true.");
    }
    if (!moveGripper(close_target, true)) {
      RCLCPP_WARN(get_logger(), "pickLatestTarget failed: closing gripper failed.");
      return false;
    }
    if (!liftAfterGrasp()) {
      RCLCPP_WARN(get_logger(), "pickLatestTarget failed: lift after grasp failed.");
      return false;
    }

    RCLCPP_INFO(get_logger(), "pickLatestTarget succeeded.");
    return true;
  }

  bool autoCooldownElapsed() const
  {
    return (now() - last_auto_execute_time_).seconds() >= auto_execute_cooldown_sec_;
  }

  bool finishGraspPose()
  {
    geometry_msgs::msg::PoseStamped target;
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      if (!have_pending_target_) {
        RCLCPP_WARN(get_logger(), "finishGraspPose failed: no pending target available.");
        return false;
      }
      target = pending_target_;
    }

    if (!isTargetInsideWorkspace(target)) {
      RCLCPP_WARN(
        get_logger(),
        "finishGraspPose rejected target outside workspace: x=%.3f y=%.3f z=%.3f",
        target.pose.position.x, target.pose.position.y, target.pose.position.z);
      return false;
    }

    const auto open_target = get_parameter_or<std::string>("open_gripper_target", "open");
    const auto close_target = get_parameter_or<std::string>("close_gripper_target", "close");

    if (!moveGripper(open_target)) {
      RCLCPP_WARN(get_logger(), "finishGraspPose failed: opening gripper failed.");
      return false;
    }

    geometry_msgs::msg::PoseStamped grasp = target;
    grasp.pose.position.z += grasp_height_m_;

    const bool grasp_motion_succeeded = moveToGrasp(grasp, "grasp");
    if (!grasp_motion_succeeded && !close_gripper_on_grasp_motion_failure_) {
      RCLCPP_WARN(get_logger(), "finishGraspPose failed: grasp motion failed.");
      return false;
    }
    if (!grasp_motion_succeeded) {
      RCLCPP_WARN(
        get_logger(),
        "finishGraspPose: grasp motion failed, but closing gripper anyway because "
        "close_gripper_on_grasp_motion_failure is true.");
    }
    if (!moveGripper(close_target, true)) {
      RCLCPP_WARN(get_logger(), "finishGraspPose failed: closing gripper failed.");
      return false;
    }
    if (!liftAfterGrasp()) {
      RCLCPP_WARN(get_logger(), "finishGraspPose failed: lift after grasp failed.");
      return false;
    }

    have_pending_target_ = false;
    RCLCPP_INFO(get_logger(), "finishGraspPose succeeded.");
    return true;
  }

  bool isTargetInsideWorkspace(const geometry_msgs::msg::PoseStamped & pose) const
  {
    const auto x = pose.pose.position.x;
    const auto y = pose.pose.position.y;
    const auto z = pose.pose.position.z;
    const auto radius = std::hypot(x, y);
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
           radius <= max_target_radius_m_;
  }

  bool moveArm(const geometry_msgs::msg::PoseStamped & target, const std::string & label)
  {
    if (!arm_) {
      RCLCPP_ERROR(get_logger(), "MoveGroupInterface for arm is not initialized.");
      return false;
    }
    arm_->setStartStateToCurrentState();
    arm_->setPositionTarget(
      target.pose.position.x,
      target.pose.position.y,
      target.pose.position.z);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = static_cast<bool>(arm_->plan(plan));
    if (!planned) {
      RCLCPP_ERROR(get_logger(), "Planning failed for %s.", label.c_str());
      arm_->clearPoseTargets();
      return false;
    }
    const auto executed = arm_->execute(plan);
    arm_->clearPoseTargets();
    if (executed != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Execution failed for %s.", label.c_str());
      return false;
    }
    return true;
  }

  bool moveToGrasp(const geometry_msgs::msg::PoseStamped & target, const std::string & label)
  {
    if (lock_joint4_during_grasp_descent_) {
      return moveArmWithJoint4Locked(target, label);
    }
    if (!use_cartesian_grasp_descent_) {
      return moveArm(target, label);
    }
    return moveArmCartesianToPosition(target, label);
  }

  bool moveArmWithJoint4Locked(
    const geometry_msgs::msg::PoseStamped & target,
    const std::string & label)
  {
    if (!arm_) {
      RCLCPP_ERROR(get_logger(), "MoveGroupInterface for arm is not initialized.");
      return false;
    }

    double joint4_position = 0.0;
    if (!getCurrentJointValue("joint4", joint4_position)) {
      RCLCPP_WARN(get_logger(), "Could not read current joint4. Falling back to normal grasp motion.");
      return use_cartesian_grasp_descent_ ?
        moveArmCartesianToPosition(target, label) : moveArm(target, label);
    }

    moveit_msgs::msg::JointConstraint joint4_constraint;
    joint4_constraint.joint_name = "joint4";
    joint4_constraint.position = joint4_position;
    joint4_constraint.tolerance_above = grasp_joint4_lock_tolerance_rad_;
    joint4_constraint.tolerance_below = grasp_joint4_lock_tolerance_rad_;
    joint4_constraint.weight = 1.0;

    moveit_msgs::msg::Constraints constraints;
    constraints.name = "lock_joint4_during_grasp";
    constraints.joint_constraints.push_back(joint4_constraint);

    arm_->setStartStateToCurrentState();
    arm_->setPathConstraints(constraints);
    arm_->setPositionTarget(
      target.pose.position.x,
      target.pose.position.y,
      target.pose.position.z);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = static_cast<bool>(arm_->plan(plan));
    arm_->clearPathConstraints();
    arm_->clearPoseTargets();
    if (!planned) {
      RCLCPP_ERROR(
        get_logger(),
        "Planning failed for %s with joint4 locked at %.3f rad.",
        label.c_str(),
        joint4_position);
      return false;
    }

    const auto executed = arm_->execute(plan);
    if (executed != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Execution failed for %s with joint4 locked.", label.c_str());
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "%s succeeded with joint4 locked near %.3f rad.",
      label.c_str(),
      joint4_position);
    return true;
  }

  bool getCurrentJointValue(const std::string & joint_name, double & value) const
  {
    if (!arm_) {
      return false;
    }

    const auto names = arm_->getJointNames();
    const auto values = arm_->getCurrentJointValues();
    if (names.size() != values.size()) {
      return false;
    }

    for (size_t i = 0; i < names.size(); ++i) {
      if (names[i] == joint_name) {
        value = values[i];
        return true;
      }
    }
    return false;
  }

  bool moveArmCartesianToPosition(
    const geometry_msgs::msg::PoseStamped & target,
    const std::string & label)
  {
    if (!arm_) {
      RCLCPP_ERROR(get_logger(), "MoveGroupInterface for arm is not initialized.");
      return false;
    }

    arm_->setStartStateToCurrentState();
    auto current_pose = arm_->getCurrentPose().pose;
    current_pose.position.x = target.pose.position.x;
    current_pose.position.y = target.pose.position.y;
    current_pose.position.z = target.pose.position.z;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(current_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = arm_->computeCartesianPath(
      waypoints,
      cartesian_eef_step_m_,
      cartesian_jump_threshold_,
      trajectory);

    if (fraction < cartesian_min_fraction_) {
      RCLCPP_WARN(
        get_logger(),
        "Cartesian %s failed: fraction %.3f is below threshold %.3f.",
        label.c_str(),
        fraction,
        cartesian_min_fraction_);
      arm_->stop();
      arm_->clearPoseTargets();
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    const auto executed = arm_->execute(plan);
    arm_->stop();
    arm_->clearPoseTargets();
    if (executed != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Execution failed for Cartesian %s.", label.c_str());
      return false;
    }

    RCLCPP_INFO(get_logger(), "Cartesian %s succeeded. fraction=%.3f", label.c_str(), fraction);
    return true;
  }

  bool liftAfterGrasp()
  {
    if (!arm_) {
      RCLCPP_ERROR(get_logger(), "MoveGroupInterface for arm is not initialized.");
      return false;
    }

    if (lift_wait_sec_ > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(lift_wait_sec_));
    }

    if (use_joint_lift_pose_) {
      return moveArmToJointLiftPose();
    }

    arm_->setStartStateToCurrentState();
    const auto current_pose = arm_->getCurrentPose();
    geometry_msgs::msg::Pose lift_pose = current_pose.pose;
    lift_pose.position.z += lift_height_m_;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(lift_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = arm_->computeCartesianPath(
      waypoints,
      cartesian_eef_step_m_,
      cartesian_jump_threshold_,
      trajectory);

    if (fraction < cartesian_min_fraction_) {
      RCLCPP_WARN(
        get_logger(),
        "Post-grasp lift failed: Cartesian fraction %.3f is below threshold %.3f.",
        fraction,
        cartesian_min_fraction_);
      arm_->stop();
      arm_->clearPoseTargets();
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    const auto executed = arm_->execute(plan);
    arm_->stop();
    arm_->clearPoseTargets();
    if (executed != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Execution failed for post-grasp lift.");
      return false;
    }

    RCLCPP_INFO(get_logger(), "Post-grasp lift succeeded. fraction=%.3f", fraction);
    return true;
  }

  bool moveArmToJointLiftPose()
  {
    if (!arm_) {
      RCLCPP_ERROR(get_logger(), "MoveGroupInterface for arm is not initialized.");
      return false;
    }

    arm_->setStartStateToCurrentState();
    arm_->setJointValueTarget(
      std::map<std::string, double>{
        {"joint1", lift_joint1_rad_},
        {"joint2", lift_joint2_rad_},
        {"joint3", lift_joint3_rad_},
        {"joint4", lift_joint4_rad_}});

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = static_cast<bool>(arm_->plan(plan));
    if (!planned) {
      RCLCPP_ERROR(get_logger(), "Planning failed for post-grasp joint lift pose.");
      arm_->clearPoseTargets();
      return false;
    }

    const auto executed = arm_->execute(plan);
    arm_->clearPoseTargets();
    if (executed != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Execution failed for post-grasp joint lift pose.");
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "Post-grasp joint lift succeeded: [%.3f, %.3f, %.3f, %.3f]",
      lift_joint1_rad_,
      lift_joint2_rad_,
      lift_joint3_rad_,
      lift_joint4_rad_);
    return true;
  }

  bool moveArmToPreMotorPoseSequence()
  {
    if (close_gripper_before_pre_motor_pose_) {
      const auto close_target = get_parameter_or<std::string>("close_gripper_target", "close");
      if (!moveGripper(close_target, true)) {
        RCLCPP_WARN(get_logger(), "moveArmToPreMotorPoseSequence failed: closing gripper failed.");
        return false;
      }
    }

    if (!moveArmToJointPose(
        "pre motor pose 1",
        pre_motor_pose1_joint1_rad_,
        pre_motor_pose1_joint2_rad_,
        pre_motor_pose1_joint3_rad_,
        pre_motor_pose1_joint4_rad_))
    {
      return false;
    }

    return moveArmToJointPose(
      "pre motor pose 2",
      pre_motor_pose2_joint1_rad_,
      pre_motor_pose2_joint2_rad_,
      pre_motor_pose2_joint3_rad_,
      pre_motor_pose2_joint4_rad_);
  }

  bool dropMineAtSafeZone()
  {
    if (!moveArmToJointPose(
        "safe zone drop pose 1",
        drop_pose1_joint1_rad_,
        drop_pose1_joint2_rad_,
        drop_pose1_joint3_rad_,
        drop_pose1_joint4_rad_))
    {
      return false;
    }

    if (!moveArmToJointPose(
        "safe zone drop pose 2",
        drop_pose2_joint1_rad_,
        drop_pose2_joint2_rad_,
        drop_pose2_joint3_rad_,
        drop_pose2_joint4_rad_))
    {
      return false;
    }

    if (!moveArmToJointPose(
        "safe zone drop pose 3",
        drop_pose3_joint1_rad_,
        drop_pose3_joint2_rad_,
        drop_pose3_joint3_rad_,
        drop_pose3_joint4_rad_))
    {
      return false;
    }

    if (!moveArmToJointPose(
        "safe zone drop pose 4",
        drop_pose4_joint1_rad_,
        drop_pose4_joint2_rad_,
        drop_pose4_joint3_rad_,
        drop_pose4_joint4_rad_))
    {
      return false;
    }

    const auto open_target = get_parameter_or<std::string>("open_gripper_target", "open");
    if (!moveGripper(open_target)) {
      RCLCPP_WARN(get_logger(), "dropMineAtSafeZone failed: opening gripper failed.");
      return false;
    }

    if (!moveArmToJointPose(
        "safe zone drop return pose 3",
        drop_pose3_joint1_rad_,
        drop_pose3_joint2_rad_,
        drop_pose3_joint3_rad_,
        drop_pose3_joint4_rad_))
    {
      return false;
    }

    if (!moveArmToJointPose(
        "safe zone drop return pose 2",
        drop_pose2_joint1_rad_,
        drop_pose2_joint2_rad_,
        drop_pose2_joint3_rad_,
        drop_pose2_joint4_rad_))
    {
      return false;
    }

    if (!moveArmToJointPose(
        "safe zone drop return pose 1",
        drop_pose1_joint1_rad_,
        drop_pose1_joint2_rad_,
        drop_pose1_joint3_rad_,
        drop_pose1_joint4_rad_))
    {
      return false;
    }

    if (!moveArmToJointLiftPose()) {
      RCLCPP_WARN(get_logger(), "dropMineAtSafeZone failed: return to start pose failed.");
      return false;
    }

    const auto close_target = get_parameter_or<std::string>("close_gripper_target", "close");
    if (!moveGripper(close_target, true)) {
      RCLCPP_WARN(get_logger(), "dropMineAtSafeZone failed: closing gripper failed.");
      return false;
    }

    RCLCPP_INFO(get_logger(), "dropMineAtSafeZone succeeded.");
    return true;
  }

  bool moveArmToJointPose(
    const std::string & label,
    const double joint1,
    const double joint2,
    const double joint3,
    const double joint4)
  {
    if (!arm_) {
      RCLCPP_ERROR(get_logger(), "MoveGroupInterface for arm is not initialized.");
      return false;
    }

    arm_->setStartStateToCurrentState();
    arm_->setJointValueTarget(
      std::map<std::string, double>{
        {"joint1", joint1},
        {"joint2", joint2},
        {"joint3", joint3},
        {"joint4", joint4}});

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = static_cast<bool>(arm_->plan(plan));
    if (!planned) {
      RCLCPP_ERROR(get_logger(), "Planning failed for %s.", label.c_str());
      arm_->clearPoseTargets();
      return false;
    }

    const auto executed = arm_->execute(plan);
    arm_->clearPoseTargets();
    if (executed != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Execution failed for %s.", label.c_str());
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "%s succeeded: [%.3f, %.3f, %.3f, %.3f]",
      label.c_str(),
      joint1,
      joint2,
      joint3,
      joint4);
    return true;
  }

  bool moveGripper(const std::string & named_target, const bool is_close = false)
  {
    if (!gripper_) {
      RCLCPP_ERROR(get_logger(), "MoveGroupInterface for gripper is not initialized.");
      return false;
    }
    if (is_close && use_custom_close_position_) {
      gripper_->setJointValueTarget(
        std::map<std::string, double>{{close_gripper_joint_name_, close_gripper_joint_position_m_}});
    } else {
      gripper_->setNamedTarget(named_target);
    }
    if (is_close && assume_close_after_timeout_) {
      gripper_->asyncMove();
      std::this_thread::sleep_for(std::chrono::duration<double>(close_gripper_timeout_sec_));
      gripper_->stop();
      RCLCPP_INFO(
        get_logger(),
        "Assuming gripper close completed after %.2f seconds.",
        close_gripper_timeout_sec_);
      return true;
    }

    const auto result = gripper_->move();
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Gripper move to '%s' failed.", named_target.c_str());
      return false;
    }
    return true;
  }

  std::string target_pose_topic_;
  bool execute_on_target_{false};
  bool have_target_{false};
  bool pending_auto_execute_{false};
  bool executing_{false};
  double approach_height_m_{0.06};
  double grasp_height_m_{0.015};
  double lift_height_m_{0.08};
  double lift_wait_sec_{1.0};
  bool use_joint_lift_pose_{true};
  double lift_joint1_rad_{-1.4835298642};
  double lift_joint2_rad_{-0.1919862177};
  double lift_joint3_rad_{-0.4712388980};
  double lift_joint4_rad_{2.181661565};
  bool move_to_start_pose_on_startup_{true};
  double startup_start_pose_delay_sec_{3.0};
  double pre_motor_pose1_joint1_rad_{-1.5358897418};
  double pre_motor_pose1_joint2_rad_{0.3316125579};
  double pre_motor_pose1_joint3_rad_{0.1919862177};
  double pre_motor_pose1_joint4_rad_{0.4363323130};
  double pre_motor_pose2_joint1_rad_{-1.5358897418};
  double pre_motor_pose2_joint2_rad_{1.0297442587};
  double pre_motor_pose2_joint3_rad_{0.2268928028};
  double pre_motor_pose2_joint4_rad_{0.1570796327};
  double drop_pose1_joint1_rad_{-2.3911010752};
  double drop_pose1_joint2_rad_{-0.2617993878};
  double drop_pose1_joint3_rad_{0.0};
  double drop_pose1_joint4_rad_{1.6929693744};
  double drop_pose2_joint1_rad_{-2.3911010752};
  double drop_pose2_joint2_rad_{0.5410520681};
  double drop_pose2_joint3_rad_{-0.6457718232};
  double drop_pose2_joint4_rad_{1.6929693744};
  double drop_pose3_joint1_rad_{-2.3911010752};
  double drop_pose3_joint2_rad_{0.9599310886};
  double drop_pose3_joint3_rad_{-0.5934119457};
  double drop_pose3_joint4_rad_{1.2566370614};
  double drop_pose4_joint1_rad_{-2.3911010752};
  double drop_pose4_joint2_rad_{1.4311699866};
  double drop_pose4_joint3_rad_{-0.7679448709};
  double drop_pose4_joint4_rad_{0.7504915784};
  double cartesian_eef_step_m_{0.01};
  double cartesian_jump_threshold_{0.0};
  double cartesian_min_fraction_{0.9};
  bool use_cartesian_grasp_descent_{true};
  bool lock_joint4_during_grasp_descent_{true};
  double grasp_joint4_lock_tolerance_rad_{0.08};
  bool assume_close_after_timeout_{true};
  double close_gripper_timeout_sec_{3.0};
  bool close_gripper_on_grasp_motion_failure_{true};
  bool close_gripper_before_pre_motor_pose_{true};
  bool use_custom_close_position_{true};
  std::string close_gripper_joint_name_{"gripper_left_joint"};
  double close_gripper_joint_position_m_{0.0045};
  double auto_execute_cooldown_sec_{10.0};
  double max_target_radius_m_{0.36};
  rclcpp::Time last_auto_execute_time_;

  std::mutex target_mutex_;
  geometry_msgs::msg::PoseStamped latest_target_;
  bool have_pending_target_{false};
  geometry_msgs::msg::PoseStamped pending_target_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pick_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr finish_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr return_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pre_motor_pose_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr drop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_service_;
  rclcpp::TimerBase::SharedPtr auto_timer_;
  rclcpp::TimerBase::SharedPtr startup_pose_timer_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ColorGraspMoveIt>(rclcpp::NodeOptions());

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  node->initMoveGroups();

  spinner.join();
  rclcpp::shutdown();
  return 0;
}
