#include <cmath>
#include "brain_tree.h"
#include "brain.h"
#include "utils/math.h"
#include "utils/print.h"
#include "utils/misc.h"
#include "std_msgs/msg/string.hpp"

/**
 * Here, a macro definition is used to reduce the amount of code in RegisterBuilder.
 * The effect after expanding REGISTER_BUILDER(Test) is as follows:
 * factory.registerBuilder<Test>(  \
 *      "Test",                    \
 *      [this](const string& name, const NodeConfig& config) { return make_unique<Test>(name, config, brain); });
 */
#define REGISTER_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [this](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });

void BrainTree::init()
{
    BehaviorTreeFactory factory;

    // Action Nodes for dribble.xml
    REGISTER_BUILDER(SimpleChase)
    REGISTER_BUILDER(SetVelocity)
    REGISTER_BUILDER(CamTrackBall)
    REGISTER_BUILDER(CamFindBall)
    REGISTER_BUILDER(CamScanField)
    REGISTER_BUILDER(Adjust)

    factory.registerBehaviorTreeFromFile(brain->config->treeFilePath);
    tree = factory.createTree("MainTree");

    // init blackboard entry
    initEntry();
}

void BrainTree::initEntry()
{
    setEntry<bool>("ball_location_known", false);
    setEntry<double>("ball_range", 0);
    setEntry<int>("control_state", 0);
}

void BrainTree::tick()
{
    tree.tickOnce();
}

NodeStatus SetVelocity::tick()
{
    double x, y, theta;
    getInput("x", x);
    getInput("y", y);
    getInput("theta", theta);

    brain->client->setVelocity(x, y, theta);
    return NodeStatus::SUCCESS;
}

NodeStatus CamTrackBall::tick()
{
    double pitch, yaw;
    if (!brain->data->ballDetected)
    {
        pitch = brain->data->ball.pitchToRobot;
        yaw = brain->data->ball.yawToRobot;
    }
    else
    {
        const double pixTolerance = 10;

        double deltaX = mean(brain->data->ball.boundingBox.xmax, brain->data->ball.boundingBox.xmin) - brain->config->camPixX / 2;
        double deltaY = mean(brain->data->ball.boundingBox.ymax, brain->data->ball.boundingBox.ymin) - brain->config->camPixY * 2 / 3;

        if (std::fabs(deltaX) < pixTolerance && std::fabs(deltaY) < pixTolerance)
        {
            return NodeStatus::SUCCESS;
        }

        double smoother = 1.5;
        double deltaYaw = deltaX / brain->config->camPixX * brain->config->camAngleX / smoother;
        double deltaPitch = deltaY / brain->config->camPixY * brain->config->camAngleY / smoother;

        pitch = brain->data->headPitch + deltaPitch;
        yaw = brain->data->headYaw - deltaYaw;
    }

    brain->client->moveHead(pitch, yaw);
    return NodeStatus::SUCCESS;
}

CamFindBall::CamFindBall(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
{
    double lowPitch = 0.8;
    double highPitch = 0.3;
    double leftYaw = 0.55;
    double rightYaw = -0.55;

    _cmdSequence[0][0] = lowPitch;
    _cmdSequence[0][1] = leftYaw;
    _cmdSequence[1][0] = lowPitch;
    _cmdSequence[1][1] = 0;
    _cmdSequence[2][0] = lowPitch;
    _cmdSequence[2][1] = rightYaw;
    _cmdSequence[3][0] = highPitch;
    _cmdSequence[3][1] = rightYaw;
    _cmdSequence[4][0] = highPitch;
    _cmdSequence[4][1] = 0;
    _cmdSequence[5][0] = highPitch;
    _cmdSequence[5][1] = leftYaw;

    _cmdIndex = 0;
    _cmdIntervalMSec = 800;
    _cmdRestartIntervalMSec = 50000;
    _timeLastCmd = brain->get_clock()->now();
}

NodeStatus CamFindBall::tick()
{
    if (brain->data->ballDetected)
    {
        return NodeStatus::SUCCESS;
    }

    auto curTime = brain->get_clock()->now();
    auto timeSinceLastCmd = (curTime - _timeLastCmd).nanoseconds() / 1e6;
    if (timeSinceLastCmd < _cmdIntervalMSec)
    {
        return NodeStatus::SUCCESS;
    }
    else if (timeSinceLastCmd > _cmdRestartIntervalMSec)
    {
        _cmdIndex = 0;
    }
    else
    {
        _cmdIndex = (_cmdIndex + 1) % (sizeof(_cmdSequence) / sizeof(_cmdSequence[0]));
    }

    brain->client->moveHead(_cmdSequence[_cmdIndex][0], _cmdSequence[_cmdIndex][1]);
    _timeLastCmd = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus CamScanField::tick()
{
    auto sec = brain->get_clock()->now().seconds();
    auto msec = static_cast<unsigned long long>(sec * 1000);
    double lowPitch, highPitch, leftYaw, rightYaw;
    getInput("low_pitch", lowPitch);
    getInput("high_pitch", highPitch);
    getInput("left_yaw", leftYaw);
    getInput("right_yaw", rightYaw);
    int msecCycle;
    getInput("msec_cycle", msecCycle);

    int cycleTime = msec % msecCycle;
    double pitch = cycleTime > (msecCycle / 2.0) ? lowPitch : highPitch;
    double yaw = cycleTime < (msecCycle / 2.0) ? (leftYaw - rightYaw) * (2.0 * cycleTime / msecCycle) + rightYaw : (leftYaw - rightYaw) * (2.0 * (msecCycle - cycleTime) / msecCycle) + rightYaw;

    brain->client->moveHead(pitch, yaw);
    return NodeStatus::SUCCESS;
}

// Simplified Adjust implementation for kick alignment
NodeStatus Adjust::tick()
{
    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        return NodeStatus::SUCCESS;
    }

    // Get input parameters (simplified version from refer/brain_tree.cpp)
    double vxLimit, vyLimit, vthetaLimit, maxRange, minRange;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("max_range", maxRange);
    getInput("min_range", minRange);
    
    string position;
    getInput("position", position);

    double vx = 0, vy = 0, vtheta = 0;
    
    // Calculate kick direction (simplified - just kick forward for offense, backwards for defense)
    double kickDir = (position == "defense") ? 
        atan2(brain->data->ball.posToField.y, brain->data->ball.posToField.x + 4.5) :  // towards own goal
        atan2(-brain->data->ball.posToField.y, 4.5 - brain->data->ball.posToField.x);  // towards opponent goal
    
    // Calculate robot-ball angle in field frame
    double dir_rb_f = atan2(
        brain->data->ball.posToField.y - brain->data->robotPoseToField.y,
        brain->data->ball.posToField.x - brain->data->robotPoseToField.x
    );
    
    double deltaDir = toPInPI(kickDir - dir_rb_f);
    double dir = deltaDir > 0 ? -1.0 : 1.0;  // Circle direction
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;

    // Circular trajectory around the ball
    double s = 0.4;  // tangential speed
    double r = 0.8;  // turning rate
    vx = -s * dir * sin(ballYaw);
    if (ballRange > maxRange)
        vx += 0.1;  // Move closer
    if (ballRange < minRange)
        vx -= 0.1;  // Move away
    vy = s * dir * cos(ballYaw);
    vtheta = (ballYaw - dir * s) / r;

    // Apply limits
    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

    brain->client->setVelocity(vx, vy, vtheta);
    return NodeStatus::SUCCESS;
}

NodeStatus SimpleChase::tick()
{
    double stopDist, stopAngle, vyLimit, vxLimit;
    getInput("stop_dist", stopDist);
    getInput("stop_angle", stopAngle);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);

    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    double vx = brain->data->ball.posToRobot.x;
    double vy = brain->data->ball.posToRobot.y;
    double vtheta = brain->data->ball.yawToRobot * 2.0;

    double linearFactor = 1 / (1 + exp(3 * (brain->data->ball.range * fabs(brain->data->ball.yawToRobot)) - 3));
    vx *= linearFactor;
    vy *= linearFactor;

    vx = cap(vx, vxLimit, -0.1);
    vy = cap(vy, vyLimit, -vyLimit);

    if (brain->data->ball.range < stopDist)
    {
        vx = 0;
        vy = 0;
    }

    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}
