// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from linkattacher_msgs:srv/Attach.idl
// generated code does not contain a copyright notice

#ifndef LINKATTACHER_MSGS__SRV__DETAIL__ATTACH__BUILDER_HPP_
#define LINKATTACHER_MSGS__SRV__DETAIL__ATTACH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "linkattacher_msgs/srv/detail/attach__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace linkattacher_msgs
{

namespace srv
{

namespace builder
{

class Init_Attach_Request_flag
{
public:
  explicit Init_Attach_Request_flag(::linkattacher_msgs::srv::Attach_Request & msg)
  : msg_(msg)
  {}
  ::linkattacher_msgs::srv::Attach_Request flag(::linkattacher_msgs::srv::Attach_Request::_flag_type arg)
  {
    msg_.flag = std::move(arg);
    return std::move(msg_);
  }

private:
  ::linkattacher_msgs::srv::Attach_Request msg_;
};

class Init_Attach_Request_robot_id
{
public:
  Init_Attach_Request_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Attach_Request_flag robot_id(::linkattacher_msgs::srv::Attach_Request::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_Attach_Request_flag(msg_);
  }

private:
  ::linkattacher_msgs::srv::Attach_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::linkattacher_msgs::srv::Attach_Request>()
{
  return linkattacher_msgs::srv::builder::Init_Attach_Request_robot_id();
}

}  // namespace linkattacher_msgs


namespace linkattacher_msgs
{

namespace srv
{

namespace builder
{

class Init_Attach_Response_success
{
public:
  Init_Attach_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::linkattacher_msgs::srv::Attach_Response success(::linkattacher_msgs::srv::Attach_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::linkattacher_msgs::srv::Attach_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::linkattacher_msgs::srv::Attach_Response>()
{
  return linkattacher_msgs::srv::builder::Init_Attach_Response_success();
}

}  // namespace linkattacher_msgs

#endif  // LINKATTACHER_MSGS__SRV__DETAIL__ATTACH__BUILDER_HPP_
