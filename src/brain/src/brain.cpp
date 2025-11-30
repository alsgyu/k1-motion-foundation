#include <iostream>
#include <string>
#include "brain.h"
#include "utils/print.h"
#include "utils/math.h"

using namespace std;
using std::placeholders::_1;

Brain::Brain() : rclcpp::Node("brain_node")
{
    // Minimal parameters for dribble.xml
    declare_parameter<double>("robot.robot_height", 1.0);
    declare_parameter<string>("tree_file_path", "");
}

void Brain::init()
{
    // Make sure to load the configuration first
    config = std::make_shared<BrainConfig>();
    loadConfig();

    data = std::make_shared<BrainData>();
    tree = std::make_shared<BrainTree>(this);
    client = std::make_shared<RobotClient>(this);

    tree->init();
    client->init();

    // Subscribe to essential topics only
    detectionsSubscription = create_subscription<vision_interface::msg::Detections>("/booster_vision/detection", 1, bind(&Brain::detectionsCallback, this, _1));
    lowStateSubscription = create_subscription<booster_interface::msg::LowState>("/low_state", 1, bind(&Brain::lowStateCallback, this, _1));
    odometerSubscription = create_subscription<booster_interface::msg::Odometer>("/booster_motion/odometer", 1, bind(&Brain::odometerCallback, this, _1));
}

void Brain::loadConfig()
{
    get_parameter("robot.robot_height", config->robotHeight);
    get_parameter("tree_file_path", config->treeFilePath);
    config->handle();
}

void Brain::tick()
{
    updateBallMemory();
    tree->tick();
}


void Brain::updateBallMemory()
{

    double secs = msecsSince(data->ball.timePoint) / 1000;
    
    double ballMemTimeout;
    get_parameter("strategy.ball_memory_timeout", ballMemTimeout);

    if (secs > ballMemTimeout) 
    { 
        tree->setEntry<bool>("ball_location_known", false);
        tree->setEntry<bool>("ball_out", false); 
    }

    
    updateRelativePos(data->ball);
    updateRelativePos(data->tmBall);
    tree->setEntry<double>("ball_range", data->ball.range);



    log->setTimeNow();
    log->logBall(
        "field/ball", 
        data->ball.posToField, 
        data->ballDetected ? 0x00FF00FF : 0x006600FF,
        data->ballDetected,
        tree->getEntry<bool>("ball_location_known")
        );
    log->logBall(
        "field/tmBall", 
        data->tmBall.posToField, 
        0xFFFF00FF,
        tree->getEntry<bool>("tm_ball_pos_reliable"),
        tree->getEntry<bool>("tm_ball_pos_reliable")
        );
}

void Brain::detectionsCallback(const vision_interface::msg::Detections &msg)
{
    auto gameObjects = getGameObjects(msg);

    vector<GameObject> balls;
    for (int i = 0; i < gameObjects.size(); i++)
    {
        const auto &obj = gameObjects[i];
        if (obj.label == "Ball")
            balls.push_back(obj);
    }

    detectProcessBalls(balls);
}

void Brain::lowStateCallback(const booster_interface::msg::LowState &msg)
{
    data->headYaw = msg.motor_state_serial[0].q;
    data->headPitch = msg.motor_state_serial[1].q;
}

void Brain::odometerCallback(const booster_interface::msg::Odometer &msg)
{
    data->robotPoseToOdom.x = msg.x * config->robotOdomFactor;
    data->robotPoseToOdom.y = msg.y * config->robotOdomFactor;
    data->robotPoseToOdom.theta = msg.theta;

    // Update robot position in Field coordinates based on Odom info
    transCoord(
        data->robotPoseToOdom.x, data->robotPoseToOdom.y, data->robotPoseToOdom.theta,
        data->odomToField.x, data->odomToField.y, data->odomToField.theta,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta);
}

vector<GameObject> Brain::getGameObjects(const vision_interface::msg::Detections &detections)
{
    vector<GameObject> res;

    auto timestamp = detections.header.stamp;
    rclcpp::Time timePoint(timestamp.sec, timestamp.nanosec);

    for (int i = 0; i < detections.detected_objects.size(); i++)
    {
        auto obj = detections.detected_objects[i];
        GameObject gObj;

        gObj.timePoint = timePoint;
        gObj.label = obj.label;

        gObj.boundingBox.xmax = obj.xmax;
        gObj.boundingBox.xmin = obj.xmin;
        gObj.boundingBox.ymax = obj.ymax;
        gObj.boundingBox.ymin = obj.ymin;
        gObj.confidence = obj.confidence;

        if (obj.position.size() > 0 && !(obj.position[0] == 0 && obj.position[1] == 0))
        {
            gObj.posToRobot.x = obj.position[0];
            gObj.posToRobot.y = obj.position[1];
        }
        else
        {
            gObj.posToRobot.x = obj.position_projection[0];
            gObj.posToRobot.y = obj.position_projection[1];
        }

        gObj.range = norm(gObj.posToRobot.x, gObj.posToRobot.y);
        gObj.yawToRobot = atan2(gObj.posToRobot.y, gObj.posToRobot.x);
        gObj.pitchToRobot = atan2(config->robotHeight, gObj.range);

        res.push_back(gObj);
    }

    return res;
}

void Brain::detectProcessBalls(const vector<GameObject> &ballObjs)
{
    const double confidenceValve = 0.35;

    double bestConfidence = 0;
    int indexRealBall = -1;

    // Find the most likely real ball
    for (int i = 0; i < ballObjs.size(); i++)
    {
        auto ballObj = ballObjs[i];

        if (ballObj.confidence < confidenceValve)
            continue;

        if (ballObj.posToRobot.x < -0.5 || ballObj.posToRobot.x > 10.0)
            continue;

        if (ballObj.confidence > bestConfidence)
        {
            bestConfidence = ballObj.confidence;
            indexRealBall = i;
        }
    }

    if (indexRealBall >= 0)
    {
        data->ballDetected = true;
        data->ball = ballObjs[indexRealBall];
        tree->setEntry<bool>("ball_location_known", true);
    }
    else
    {
        data->ballDetected = false;
        data->ball.boundingBox.xmin = 0;
        data->ball.boundingBox.xmax = 0;
        data->ball.boundingBox.ymin = 0;
        data->ball.boundingBox.ymax = 0;
        data->ball.confidence = 0;
    }

}