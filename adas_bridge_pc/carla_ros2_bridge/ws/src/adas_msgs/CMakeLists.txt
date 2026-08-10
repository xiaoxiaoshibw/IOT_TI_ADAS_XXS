cmake_minimum_required(VERSION 3.16)
project(adas_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrackedObject.msg"
  "msg/TrackedObjectArray.msg"
  "msg/LaneState.msg"
  "msg/TrajectoryPoint.msg"
  "msg/Trajectory.msg"
  "msg/LateralCommand.msg"
  "msg/LongitudinalCommand.msg"
  "msg/Control.msg"
  "msg/ActuationCommand.msg"
  "msg/SteeringReport.msg"
  "msg/BehaviorState.msg"
  "msg/MapSign.msg"
  "msg/AebStatus.msg"
  "msg/GateStatus.msg"
  "msg/Heartbeat.msg"
  "msg/SafetyStatus.msg"
  "msg/LaneConnection.msg"
  "msg/MapLane.msg"
  "msg/LaneGraph.msg"
  "msg/NavigationStatus.msg"
  "msg/McuStatus.msg"
  "srv/ChangeCommandSource.srv"
  "srv/SetNavigationGoal.srv"
  "srv/CancelNavigation.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_package()
