#pragma once

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <eeros/logger/Logger.hpp>
#include <eeros/control/ros2/EerosRosTools.hpp>

namespace eeros {
namespace control {

#pragma message("WARNING: You are using an experimental class that has not been fully tested: " __FILE__)

/**
 * This class is used to publish a tf frame to the global tf tree.
 * 
 * This is only for static transformations.
 * It will not publish the transformation continuously.
 * For dynamic transformations, see the class: eeros::control::Ros2TransformBroadcaster.hpp
 */
class Ros2StaticTransformBroadcaster {
public:
    static Ros2StaticTransformBroadcaster& getInstance(rclcpp::Node::SharedPtr node, uint32_t queueSize = 1000) {
        static Ros2StaticTransformBroadcaster instance(node, queueSize);
        return instance;
    }

    /**
     * Broadcast a single static Tf
     * If you need to send multiple TFs, use the overload feature (better performance).
     * 
     * @param tf: The static tf to broadcast
     */
    void addTf(const geometry_msgs::msg::TransformStamped& tf) {
        static_tf_broadcaster->sendTransform(tf);
        tfs.push_back(tf);
    }

    /**
     * Broadcast a list of static Tf
     * 
     * @param tf: The static tf list to broadcast
     */
    void addTf(const std::vector<geometry_msgs::msg::TransformStamped>& tf) {
        static_tf_broadcaster->sendTransform(tf);
        tfs.insert(tfs.end(), tf.begin(), tf.end());
    }

    /**
     * Returns a list of all published tf's
     */
    std::vector<geometry_msgs::msg::TransformStamped> getTfs() const {
        return tfs;
    }

private:
    Ros2StaticTransformBroadcaster(rclcpp::Node::SharedPtr node, uint32_t queueSize)
        : node(node),
          log(eeros::logger::Logger::getLogger('t')) {
        if (!node) {
            throw std::runtime_error("Node must not be null");
        }
        static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node, queueSize);
        log.info() << "ROS TF static broadcaster generated on node '" << node->get_name() << "'";
    }

    Ros2StaticTransformBroadcaster(const Ros2StaticTransformBroadcaster&) = delete;
    Ros2StaticTransformBroadcaster& operator=(const Ros2StaticTransformBroadcaster&) = delete;

    rclcpp::Node::SharedPtr node;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;
    eeros::logger::Logger log;
    std::vector<geometry_msgs::msg::TransformStamped> tfs;
};

} /* END Namespace: control */
} /* END Namespace: eeros */