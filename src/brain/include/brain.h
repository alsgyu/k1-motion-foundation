#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <vision_interface/msg/detections.hpp>

#include "booster_interface/msg/low_state.hpp"
#include "booster_interface/msg/odometer.hpp"

#include "brain_config.h"
#include "brain_data.h"
#include "brain_tree.h"
#include "robot_client.h"

using namespace std;

/**
 * The Brain class, minimal version for dribble.xml
 */
class Brain : public rclcpp::Node
{
public:
    std::shared_ptr<BrainConfig> config;
    std::shared_ptr<BrainData> data;
    std::shared_ptr<RobotClient> client;
    std::shared_ptr<BrainTree> tree;

    Brain();
    void init();
    void tick();

private:
    void loadConfig();
    void updateBallMemory();
    
    // Essential callbacks only
    void detectionsCallback(const vision_interface::msg::Detections &msg);
    void lowStateCallback(const booster_interface::msg::LowState &msg);
    void odometerCallback(const booster_interface::msg::Odometer &msg);
    
    vector<GameObject> getGameObjects(const vision_interface::msg::Detections &msg);
    void detectProcessBalls(const vector<GameObject> &ballObjs);
    
    // Helper functions
    void updateFieldPos(GameObject &obj);
    double msecsSince(rclcpp::Time time);

    rclcpp::Subscription<vision_interface::msg::Detections>::SharedPtr detectionsSubscription;
    rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr lowStateSubscription;
    rclcpp::Subscription<booster_interface::msg::Odometer>::SharedPtr odometerSubscription;
};
