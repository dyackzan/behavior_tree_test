#include <iostream>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include <string>
#include <functional>
#include <filesystem>

using namespace BT;

// SyncActionNode (synchronous action) with an input port.
class CreateAnyPoseStampedVector : public SyncActionNode {
public:
    // If your Node has ports, you must use this constructor signature
    CreateAnyPoseStampedVector(const std::string &name, const NodeConfig &config) : SyncActionNode(name, config) {}

    // It is mandatory to define this STATIC method.
    static PortsList providedPorts() {
      return {OutputPort<std::vector<BT::Any>>("pose_stamped_vector", "{vector}",
                                               "this is a BT::Any object on the blackboard")};
    }

    // Override the virtual function tick()
    NodeStatus tick() override {
      auto pose1 = geometry_msgs::msg::PoseStamped();
      pose1.pose.position.x = 2;
      auto pose2 = geometry_msgs::msg::PoseStamped();
      pose2.pose.position.x = 3;

      std::vector<BT::Any> out = {BT::Any(pose1), BT::Any(pose2)};

      setOutput("pose_stamped_vector", out);

      return NodeStatus::SUCCESS;
    }
};

class ForEachAny : public DecoratorNode {
public:
    ForEachAny(const std::string &name, const BT::NodeConfiguration &config) : BT::DecoratorNode(name, config) { }

    // It is mandatory to define this STATIC method.
    static PortsList providedPorts() {
      return {InputPort<std::vector<BT::Any>>(kPortIDInput, "{any_vec}", "Input BT::Any vector to loop over"),
              OutputPort<BT::Any>(kPortIDOutput, "{any_val}")};
    }

    NodeStatus tick() {
      const auto input_vector = getInput<std::vector<BT::Any>>(kPortIDInput);

      if (!input_vector.has_value()) {
        std::cout << input_vector.error() << std::endl;
        return BT::NodeStatus::FAILURE;
      }

      setStatus(BT::NodeStatus::RUNNING);

      if (current_index_ < input_vector.value().size()) {
        setOutput<BT::Any>(kPortIDOutput, input_vector.value().at(current_index_));

        const BT::NodeStatus child_state = child_node_->executeTick();

        switch (child_state) {
          case BT::NodeStatus::SUCCESS: {
            current_index_++;
            haltChild();
            return BT::NodeStatus::RUNNING;
          }

          case BT::NodeStatus::FAILURE: {
            current_index_ = 0;
            haltChild();
            return (BT::NodeStatus::FAILURE);
          }

          case BT::NodeStatus::RUNNING: {
            return BT::NodeStatus::RUNNING;
          }

          default: {
            throw BT::LogicError("A child node must never return IDLE");
          }
        }
      }

      current_index_ = 0;
      return BT::NodeStatus::SUCCESS;
    }

    void halt() {
      current_index_ = 0;
      DecoratorNode::halt();
    }

    static constexpr auto kPortIDInput = "vector_in";
    static constexpr auto kPortIDOutput = "out";
    std::size_t current_index_{ 0 };
};

// SyncActionNode (synchronous action) with an input port.
class AnyToPoseStamped : public SyncActionNode {
public:
    // If your Node has ports, you must use this constructor signature
    AnyToPoseStamped(const std::string &name, const NodeConfig &config) : SyncActionNode(name, config) {}

    // It is mandatory to define this STATIC method.
    static PortsList providedPorts() {
      return {InputPort<geometry_msgs::msg::PoseStamped>("input", "{any_val}", "this is a BT::Any object on the blackboard")};
    }

    // Override the virtual function tick()
    NodeStatus tick() override {
      Expected<geometry_msgs::msg::PoseStamped> msg = getInput<geometry_msgs::msg::PoseStamped>("input");

      // Check if expected is valid. If not, throw its error
      if (!msg) {
        throw BT::RuntimeError("missing required input [message]: ", msg.error());
      }
//      auto any_msg = config().blackboard->getAnyLocked(msg.value());
//      auto pose_msg = any_msg->cast<geometry_msgs::msg::PoseStamped>();

      // use the method value() to extract the valid message.
//      std::cout << "Pose name on blackboard is: " << msg.value() << std::endl;
      std::cout << "pose.position.x: " << msg.value().pose.position.x << std::endl;
      return NodeStatus::SUCCESS;
    }
};



int main() {
  BehaviorTreeFactory factory;
  BT::NodeConfiguration config_;
  config_.blackboard = BT::Blackboard::create();

  factory.registerNodeType<CreateAnyPoseStampedVector>("CreateAnyPoseStampedVector");
  factory.registerNodeType<ForEachAny>("ForEachAny");
  factory.registerNodeType<AnyToPoseStamped>("AnyToPoseStamped");

  auto tree = factory.createTreeFromFile("./my_tree.xml", config_.blackboard);
  tree.tickWhileRunning();

  return 0;

}