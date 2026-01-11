// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from linkattacher_msgs:srv/Attach.idl
// generated code does not contain a copyright notice

#ifndef LINKATTACHER_MSGS__SRV__DETAIL__ATTACH__STRUCT_HPP_
#define LINKATTACHER_MSGS__SRV__DETAIL__ATTACH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__linkattacher_msgs__srv__Attach_Request __attribute__((deprecated))
#else
# define DEPRECATED__linkattacher_msgs__srv__Attach_Request __declspec(deprecated)
#endif

namespace linkattacher_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Attach_Request_
{
  using Type = Attach_Request_<ContainerAllocator>;

  explicit Attach_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->flag = false;
    }
  }

  explicit Attach_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->flag = false;
    }
  }

  // field types and members
  using _robot_id_type =
    int32_t;
  _robot_id_type robot_id;
  using _flag_type =
    bool;
  _flag_type flag;

  // setters for named parameter idiom
  Type & set__robot_id(
    const int32_t & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__flag(
    const bool & _arg)
  {
    this->flag = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    linkattacher_msgs::srv::Attach_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const linkattacher_msgs::srv::Attach_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<linkattacher_msgs::srv::Attach_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<linkattacher_msgs::srv::Attach_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      linkattacher_msgs::srv::Attach_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<linkattacher_msgs::srv::Attach_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      linkattacher_msgs::srv::Attach_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<linkattacher_msgs::srv::Attach_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<linkattacher_msgs::srv::Attach_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<linkattacher_msgs::srv::Attach_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__linkattacher_msgs__srv__Attach_Request
    std::shared_ptr<linkattacher_msgs::srv::Attach_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__linkattacher_msgs__srv__Attach_Request
    std::shared_ptr<linkattacher_msgs::srv::Attach_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Attach_Request_ & other) const
  {
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->flag != other.flag) {
      return false;
    }
    return true;
  }
  bool operator!=(const Attach_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Attach_Request_

// alias to use template instance with default allocator
using Attach_Request =
  linkattacher_msgs::srv::Attach_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace linkattacher_msgs


#ifndef _WIN32
# define DEPRECATED__linkattacher_msgs__srv__Attach_Response __attribute__((deprecated))
#else
# define DEPRECATED__linkattacher_msgs__srv__Attach_Response __declspec(deprecated)
#endif

namespace linkattacher_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Attach_Response_
{
  using Type = Attach_Response_<ContainerAllocator>;

  explicit Attach_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit Attach_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    linkattacher_msgs::srv::Attach_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const linkattacher_msgs::srv::Attach_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<linkattacher_msgs::srv::Attach_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<linkattacher_msgs::srv::Attach_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      linkattacher_msgs::srv::Attach_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<linkattacher_msgs::srv::Attach_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      linkattacher_msgs::srv::Attach_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<linkattacher_msgs::srv::Attach_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<linkattacher_msgs::srv::Attach_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<linkattacher_msgs::srv::Attach_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__linkattacher_msgs__srv__Attach_Response
    std::shared_ptr<linkattacher_msgs::srv::Attach_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__linkattacher_msgs__srv__Attach_Response
    std::shared_ptr<linkattacher_msgs::srv::Attach_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Attach_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const Attach_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Attach_Response_

// alias to use template instance with default allocator
using Attach_Response =
  linkattacher_msgs::srv::Attach_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace linkattacher_msgs

namespace linkattacher_msgs
{

namespace srv
{

struct Attach
{
  using Request = linkattacher_msgs::srv::Attach_Request;
  using Response = linkattacher_msgs::srv::Attach_Response;
};

}  // namespace srv

}  // namespace linkattacher_msgs

#endif  // LINKATTACHER_MSGS__SRV__DETAIL__ATTACH__STRUCT_HPP_
