#include <queue>
#include <math.h>
#include <string>
#include <vector>
#include <utils.h>
#include <tf/tf.h>
#include <iostream>
#include <Quadtree.h>

#include <ros/ros.h>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <actionlib_msgs/GoalStatus.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <std_msgs/MultiArrayDimension.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/MarkerArray.h>
#include <actionlib/client/simple_action_client.h>

ros::Publisher marker_arr_pub;
ros::Publisher goal_pub;
bool goalVisible = false;
bool newGap = false;
bool odomRan = false;

auto getBox = [](quadtree::Node *node)
{
    return node->box;
};

quadtree::Quadtree<quadtree::Node *, decltype(getBox)> *tree;
quadtree::Quadtree<quadtree::Node *, decltype(getBox)> *narrowTree;
quadtree::Node *currGoal;
quadtree::Node *finalGoal;

std::vector<quadtree::Node *> localNodes;

std::string frame_str = "odom";
sensor_msgs::LaserScan::ConstPtr scanData;

float threshold;
float goalDist;
float top, left, bwidth, bheight;
bool isSet = false;

void publishTree();
void publishGoal(const Eigen::Vector3f &pose);
void treeGoalcb(const Eigen::Vector3f &pose);
void updateTree(std::vector<Eigen::Vector2i> &indices, const sensor_msgs::LaserScan::ConstPtr &msg, const Eigen::Vector3f &pose);

bool compare(const Eigen::Vector2i &a, const Eigen::Vector2i &b)
{
    if (a.x() == b.x())
        return a.y() > b.y();

    return a.x() < b.x();
}

void goalcb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    float goal_x, goal_y;
    goal_x = msg->pose.position.x;
    goal_y = msg->pose.position.y;

    Eigen::Vector2f goal(goal_x, goal_y);
    Eigen::Vector2f goalS(.2 + goal_x, goal_y);
    Eigen::Vector2f goalP(-.2 + goal_x, goal_y);

    finalGoal->p = goal;
    finalGoal->ps = goalS;
    finalGoal->pe = goalP;
}

void odomcb(const nav_msgs::Odometry::ConstPtr &msg, Eigen::Vector3f *pose)
{

    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    *pose = Eigen::Vector3f(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
    Eigen::Vector2f p(pose->x(), pose->y());

    quadtree::Box<float> box(pose->x() - JACKAL_LENGTH * 5, pose->y() - JACKAL_LENGTH * 5, JACKAL_LENGTH * 10, JACKAL_LENGTH * 10);
    std::vector<quadtree::Node *> nodes = tree->query(box);

    bool goalChanged = false;

    if (!isSet)
    {
        float gx = pose->x() + goalDist * cos(yaw);
        float gy = pose->y() + goalDist * sin(yaw);

        finalGoal->p = Eigen::Vector2f(gx, gy);
        finalGoal->ps = Eigen::Vector2f(gx - .2, gy);
        finalGoal->pe = Eigen::Vector2f(gx + .2, gy);
        finalGoal->box.left = gx - JACKAL_WIDTH / 2;
        finalGoal->box.top = gy - JACKAL_LENGTH / 2;

        isSet = true;
    }

    for (auto node : nodes)
    {

        float d = distToGap(node->ps, node->pe, p);

        if (d < .3)
        {
            node->visited = true;
            if (node->isGoal)
            {
                node->isGoal = false;
                goalChanged = true;
            }
        }
    }

    if (goalChanged || (currGoal != nullptr && !isGapAdmissible(currGoal->p, scanData, *pose)))
    {
        ROS_INFO("Goal reached! %d", (currGoal != nullptr && !isGapAdmissible(currGoal->p, scanData, *pose)));
        if (currGoal != nullptr && !isGapAdmissible(currGoal->p, scanData, *pose))
        {
        }
        else
            treeGoalcb(*pose);

        currGoal = nullptr;
    }
    else if (currGoal != nullptr)
    {
        publishGoal(*pose);
    }

    if ((finalGoal->p - p).squaredNorm() < .7 * .7)
    {
        ROS_INFO("Goal reached!");
        // exit(0);
    }

    odomRan = true;
}

void publishGoal(const Eigen::Vector3f &pose)
{

    Eigen::Vector2f vec = currGoal->ps - currGoal->pe;
    float angle = atan2(vec.y(), vec.x()) + M_PI / 2;
    float m = vec.y() / vec.x();
    float b = currGoal->ps.y() - currGoal->ps.x() * m;

    Eigen::Vector2f normP(cos(angle) + currGoal->p.x(), sin(angle) + currGoal->p.y());

    if (normP.y() < m * normP.x() + b &&
        pose.y() > m * pose.x() + b)
    {
        angle = angle + M_PI;
        angle = angle > 2 * M_PI ? angle - 2 * M_PI : angle;
    }

    if (normP.y() > m * normP.x() + b &&
        pose.y() < m * pose.x() + b)
    {
        angle = angle + M_PI;
        angle = angle > 2 * M_PI ? angle - 2 * M_PI : angle;
    }

    float target_x = currGoal->p.x();
    float target_y = currGoal->p.y();
    float yaw = angle;
    std_msgs::Float32MultiArray goalMsg;
    goalMsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    goalMsg.layout.dim[0].size = 3;
    goalMsg.layout.dim[0].stride = 1;
    goalMsg.layout.dim[0].label = "xy";

    goalMsg.data.push_back(target_x);
    goalMsg.data.push_back(target_y);
    goalMsg.data.push_back(yaw);
    // Publish
    goal_pub.publish(goalMsg);
}

void removeGapsNearPoint(const Eigen::Vector2f &point, const Eigen::Vector3f &pose)
{

    bool goalDeleted = false;
    if (!std::isinf(point.x()) && !std::isinf(point.y()))
    {
        quadtree::Box<float> box(point.x() - .05, point.y() - .05, .1, .1);
        std::vector<quadtree::Node *> overlapping = tree->query(box);

        for (auto node : overlapping)
        {

            // if ((point-node->p).squaredNorm() > JACKAL_WIDTH*JACKAL_WIDTH/4 + JACKAL_LENGTH*JACKAL_LENGTH/4)
            if ((point - node->p).squaredNorm() > JACKAL_WIDTH * JACKAL_WIDTH / 4)
                continue;

            if (node->isGoal)
                goalDeleted = true;

            // ROS_INFO("removing node because it is too close (%.2f, %.2f)", node->p.x(), node->p.y());
            tree->remove(node);
        }
    }

    if (goalDeleted)
        treeGoalcb(pose);
}

void lasercb(const sensor_msgs::LaserScan::ConstPtr &msg, Eigen::Vector3f *pose)
{

    scanData = msg;
    if (!odomRan)
    {
        return;
    }

    std::vector<Eigen::Vector4f> gaps;
    std::vector<Eigen::Vector2i> indices;

    // counter clockwise looking for rising gaps
    for (size_t i = 0; i < msg->ranges.size() - 1;)
    {

        float a1 = msg->angle_min + i * msg->angle_increment + pose->z();
        float a2 = msg->angle_min + (i + 1) * msg->angle_increment + pose->z();

        if (a1 - pose->z() < -M_PI / 2)
        {
            i++;
            continue;
        }
        if (a1 - pose->z() > M_PI / 2)
            break;

        Eigen::Vector2f p1(msg->ranges[i] * cos(a1) + pose->x(), msg->ranges[i] * sin(a1) + pose->y());
        Eigen::Vector2f p2(msg->ranges[i + 1] * cos(a2) + pose->x(), msg->ranges[i + 1] * sin(a2) + pose->y());

        removeGapsNearPoint(p1, *pose);

        if ((p1 - p2).squaredNorm() < threshold * threshold || msg->ranges[i] > msg->ranges[i + 1])
        {
            i += 1;
            continue;
        }

        size_t champ_ind = i + 1;
        float champ_dist = 10000;

        for (size_t j = i + 1; j < msg->ranges.size(); j++)
        {
            float angle = msg->angle_min + j * msg->angle_increment + pose->z();

            float deltaA = angle - a1;

            if (deltaA > 2 * M_PI)
                deltaA -= 2 * M_PI;
            else if (deltaA < -2 * M_PI)
                deltaA += 2 * M_PI;

            if (deltaA > M_PI)
                break;

            Eigen::Vector2f pk(msg->ranges[j] * cos(angle) + pose->x(), msg->ranges[j] * sin(angle) + pose->y());

            removeGapsNearPoint(pk, *pose);

            float d = (p1 - pk).squaredNorm();
            if (d < champ_dist)
            {
                champ_dist = d;
                champ_ind = j;
            }
        }

        float angle1 = msg->angle_min + i * msg->angle_increment + pose->z();
        float angle2 = msg->angle_min + champ_ind * msg->angle_increment + pose->z();
        Eigen::Vector2f ps(msg->ranges[i] * cos(angle1) + pose->x(), msg->ranges[i] * sin(angle1) + pose->y());
        Eigen::Vector2f pe(msg->ranges[champ_ind] * cos(angle2) + pose->x(), msg->ranges[champ_ind] * sin(angle2) + pose->y());
        Eigen::Vector2f mp = (ps + pe) / 2;

        if (!isGapNearScan(*pose, mp, msg))
        {

            Eigen::Vector2i ends(i, champ_ind);
            indices.push_back(ends);

            Eigen::Vector4f gap(ps.x(), ps.y(), pe.x(), pe.y());
            gaps.push_back(gap);
        }

        i = champ_ind;
    }

    // clockwise looking for descending gaps
    for (size_t i = msg->ranges.size() - 1; i > 0;)
    {
        float a1 = msg->angle_min + i * msg->angle_increment + pose->z();
        float a2 = msg->angle_min + (i - 1) * msg->angle_increment + pose->z();

        if (a1 - pose->z() < -M_PI / 2)
            break;

        if (a1 - pose->z() > M_PI / 2)
        {
            i--;
            continue;
        }
        // ROS_INFO("%lu\t%.2f", i, a1-pose->z());

        Eigen::Vector2f p1(msg->ranges[i] * cos(a1) + pose->x(), msg->ranges[i] * sin(a1) + pose->y());
        Eigen::Vector2f p2(msg->ranges[i - 1] * cos(a2) + pose->x(), msg->ranges[i - 1] * sin(a2) + pose->y());

        if ((p1 - p2).squaredNorm() < threshold * threshold || msg->ranges[i] > msg->ranges[i - 1] || std::isinf(msg->ranges[i]))
        {
            i -= 1;
            continue;
        }

        // ROS_INFO("(%.2f,%.2f) is CW gap", p1.x(),p1.y());

        size_t champ_ind = i - 1;
        float champ_dist = 10000;

        for (size_t j = i - 1; j >= 0; j--)
        {
            float angle = msg->angle_min + j * msg->angle_increment + pose->z();

            float deltaA = a1 - angle;

            if (deltaA > 2 * M_PI)
                deltaA -= 2 * M_PI;
            else if (deltaA < -2 * M_PI)
                deltaA += 2 * M_PI;

            if (deltaA > M_PI)
                break;

            Eigen::Vector2f pk(msg->ranges[j] * cos(angle) + pose->x(), msg->ranges[j] * sin(angle) + pose->y());

            float d = (p1 - pk).squaredNorm();
            if (d < champ_dist)
            {
                champ_dist = d;
                champ_ind = j;
            }

            if (j == 0)
                break;
        }

        float angle1 = msg->angle_min + i * msg->angle_increment + pose->z();
        float angle2 = msg->angle_min + champ_ind * msg->angle_increment + pose->z();
        Eigen::Vector2f ps(msg->ranges[i] * cos(angle1) + pose->x(), msg->ranges[i] * sin(angle1) + pose->y());
        Eigen::Vector2f pe(msg->ranges[champ_ind] * cos(angle2) + pose->x(), msg->ranges[champ_ind] * sin(angle2) + pose->y());
        Eigen::Vector2f mp = (ps + pe) / 2;

        if (!isGapNearScan(*pose, mp, msg))
        {

            Eigen::Vector2i ends(champ_ind, i);
            indices.push_back(ends);

            Eigen::Vector4f gap(ps.x(), ps.y(), pe.x(), pe.y());
            gaps.push_back(gap);
        }

        i = champ_ind;
    }
    // ROS_INFO("*************************");

    goalVisible = isGapAdmissible(finalGoal->p, msg, *pose);

    updateTree(indices, msg, *pose);
    // goalcb(*pose);

    if (currGoal == nullptr)
        treeGoalcb(*pose);
    else if (finalGoal == currGoal && !goalVisible)
        treeGoalcb(*pose);

    // else if (goalVisible){
    //     treeGoalcb(*pose);
    // }
    // static int c = 0;
    // if (c++ == 1)
    //     exit(0);

    publishTree();
}

void updateTree(std::vector<Eigen::Vector2i> &indices, const sensor_msgs::LaserScan::ConstPtr &msg, const Eigen::Vector3f &pose)
{

    float epsilon = .1;
    std::sort(indices.begin(), indices.end(), compare);
    std::vector<std::vector<size_t>> children;

    // TODO: make non-naive solution
    Eigen::Vector2f roboPose(pose.x(), pose.y());
    children.resize(indices.size());

    // ROS_INFO("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
    for (size_t i = 0; i < indices.size(); i++)
    {
        Eigen::Vector2i intA = indices[i];
        int left = intA.x();
        int right = intA.y();

        float angle1 = scanData->angle_min + left * scanData->angle_increment + pose.z();
        float angle2 = scanData->angle_min + right * scanData->angle_increment + pose.z();

        Eigen::Vector2f p1(scanData->ranges[left] * cos(angle1) + pose.x(), scanData->ranges[left] * sin(angle1) + pose.y());
        Eigen::Vector2f p2(scanData->ranges[right] * cos(angle2) + pose.x(), scanData->ranges[right] * sin(angle2) + pose.y());
        Eigen::Vector2f mpA = (p1 + p2) / 2;
        // ROS_INFO("%lu -- nodeA: (%.2f,%.2f)",i,mpA.x(),mpA.y());
        bool isChild = false;
        for (size_t j = 0; j < indices.size(); j++)
        {
            if (j == i)
                continue;
            Eigen::Vector2i intB = indices[j];
            if (intA.x() == intB.x() || intA.y() == intB.y())
                continue;

            left = intB.x();
            right = intB.y();

            angle1 = scanData->angle_min + left * scanData->angle_increment + pose.z();
            angle2 = scanData->angle_min + right * scanData->angle_increment + pose.z();

            p1 = Eigen::Vector2f(scanData->ranges[left] * cos(angle1) + pose.x(), scanData->ranges[left] * sin(angle1) + pose.y());
            p2 = Eigen::Vector2f(scanData->ranges[right] * cos(angle2) + pose.x(), scanData->ranges[right] * sin(angle2) + pose.y());
            Eigen::Vector2f mpB = (p1 + p2) / 2;
            // ROS_INFO("\tnodeB: (%.2f,%.2f)",mpB.x(),mpB.y());

            if (std::max(intA.x(), intB.x()) <= std::min(intA.y(), intB.y()))
            {

                if (fabs((roboPose - mpB).squaredNorm() - (roboPose - mpA).squaredNorm()) < 1e-2)
                    continue;
                else if ((roboPose - mpB).squaredNorm() > (roboPose - mpA).squaredNorm())
                {
                    // ROS_INFO("nodeA parent of nodeB");
                    if (children[i].size() == 0)
                        children[i].push_back(i);
                    children[i].push_back(j);
                }
                else
                {
                    isChild = true;
                    // ROS_INFO("nodeB parent of nodeA");
                    if (children[j].size() == 0)
                        children[j].push_back(j);
                    children[j].push_back(i);
                }
                // }
            }
        }

        if (children[i].size() == 0 && !isChild)
        {
            // ROS_INFO("index range at i=%lu is parent", i);
            children[i].push_back(i);
        }
    }

    // ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");

    for (auto it = localNodes.begin(); it != localNodes.end(); it++)
    {
        (*it)->isLocal = false;
    }

    localNodes.clear();
    bool goalChanged = false;
    for (size_t i = 0; i < children.size(); i++)
    {

        quadtree::Node *parent = nullptr;
        for (size_t j = 0; j < children[i].size(); j++)
        {

            int left = indices[children[i][j]].x();
            int right = indices[children[i][j]].y();

            float angle1 = scanData->angle_min + left * scanData->angle_increment + pose.z();
            float angle2 = scanData->angle_min + right * scanData->angle_increment + pose.z();

            Eigen::Vector2f p1(scanData->ranges[left] * cos(angle1) + pose.x(), scanData->ranges[left] * sin(angle1) + pose.y());
            Eigen::Vector2f p2(scanData->ranges[right] * cos(angle2) + pose.x(), scanData->ranges[right] * sin(angle2) + pose.y());
            Eigen::Vector2f mp = (p1 + p2) / 2;
            // ROS_INFO("parsing node (%.2f,%.2f)", mp.x(), mp.y());
            if ((p1 - p2).squaredNorm() < threshold * threshold || std::isinf(mp.x()) || std::isinf(mp.y()))
            {
                // ROS_INFO("%lu\tdogwater gap", i);
                if (j == 0)
                    break;
                continue;
            }

            // if(j == 0)
            // ROS_INFO("j==0 node is: (%.2f, %.2f)", mp.x(), mp.y());

            quadtree::Box<float> box(mp.x() - JACKAL_WIDTH / 4, mp.y() - JACKAL_LENGTH / 4, JACKAL_WIDTH / 2, JACKAL_LENGTH / 2);
            std::vector<quadtree::Node *> nodes = tree->query(box);

            // if looking at parent, check if any other nodes are close to it
            // if yes, update the node that is < .1m to it
            // if no such node exists, replace all old nodes with this one and
            // make their parents/children point to this node

            bool addFlag = true;
            quadtree::Node *tmp = nullptr;
            for (auto node : nodes)
            {
                if ((node->p - mp).norm() < 1e-1)
                {
                    // ROS_INFO("(%.2f,%.2f) too close to (%.2f,%.2f)",
                    //     mp.x(), mp.y(),node->p.x(), node->p.y());
                    addFlag = false;
                    tmp = node;
                    break;
                }
            }

            if (!addFlag)
            {
                if (j == 0)
                {
                    if (tmp != nullptr)
                        parent = tmp;
                    // break;
                }
                continue;
            }

            quadtree::Node *n = new quadtree::Node();
            n->p = mp;
            n->ps = p1;
            n->pe = p2;

            n->box.left = std::min(p1.x(), p2.x());
            n->box.top = std::min(p1.y(), p2.y());
            n->box.width = fabs(p1.x() - p2.x());
            n->box.height = fabs(p1.y() - p2.y());

            n->id = tree->getCount();
            n->isGoal = false;
            n->visited = false;
            n->isLocal = true;
            n->isAdmissible = isGapAdmissible(mp, scanData, pose);

            tree->add(n);
            newGap = true;

            for (auto node : nodes)
            {

                if ((n->p - node->p).squaredNorm() > JACKAL_WIDTH * JACKAL_WIDTH / 4)
                    continue;

                if (node->isGoal)
                {
                    bool goalChanged = true;
                    currGoal = n;
                    // std::cout << currGoal << std::endl;
                    n->isGoal = node->isGoal;
                }

                n->id = node->id;

                if (node->visited)
                {
                    if ((n->p - node->p).norm() < JACKAL_LENGTH / 4)
                        n->visited = node->visited;
                }

                tree->remove(node);
            }
        }
    }
}

void publishTree()
{
    // ROS_INFO("publishing");
    visualization_msgs::MarkerArray marker_arr;

    visualization_msgs::Marker del_msg;
    del_msg.header.frame_id = frame_str;
    del_msg.header.stamp = ros::Time();
    del_msg.ns = "gap";
    del_msg.id = 1000;
    del_msg.action = visualization_msgs::Marker::DELETEALL;

    marker_arr.markers.push_back(del_msg);

    visualization_msgs::Marker line_msg;
    line_msg.header.frame_id = frame_str;
    line_msg.header.stamp = ros::Time();
    line_msg.ns = "gap";
    line_msg.id = 420;
    line_msg.type = visualization_msgs::Marker::LINE_LIST;
    line_msg.action = visualization_msgs::Marker::ADD;
    line_msg.scale.x = .1;
    line_msg.color.a = 1;
    line_msg.color.r = 243.0 / 255.0;
    line_msg.color.g = 167.0 / 255.0;
    line_msg.color.b = 18.0 / 255.0;

    visualization_msgs::Marker graph_msg;
    graph_msg.header.frame_id = frame_str;
    graph_msg.header.stamp = ros::Time();
    graph_msg.ns = "gap";
    graph_msg.id = 421;
    graph_msg.type = visualization_msgs::Marker::LINE_LIST;
    graph_msg.action = visualization_msgs::Marker::ADD;
    graph_msg.scale.x = .05;
    graph_msg.color.a = 1;
    graph_msg.color.r = 1.0;
    graph_msg.color.g = 0.0;
    graph_msg.color.b = 0.0;

    quadtree::Box<float> box(left, top, bwidth, bheight);
    std::vector<quadtree::Node *> nodes = tree->query(box);
    nodes.push_back(finalGoal);

    // std::cout << "nodes: " << std::endl;
    int count = 0;
    for (auto node : nodes)
    {

        // std::cout << "(" << nodes->x << ", " << nodes->y << ")\t";
        visualization_msgs::Marker m;
        m.header.frame_id = frame_str;
        m.header.stamp = ros::Time();
        m.ns = "gap";
        m.id = count++;
        m.type = visualization_msgs::Marker::SPHERE;
        m.action = visualization_msgs::Marker::ADD;
        m.scale.x = .35;
        m.scale.y = .35;
        m.scale.z = .35;
        m.color.a = 1;
        m.color.r = 41.0 / 255.0;
        m.color.g = 51.0 / 255.0;
        m.color.b = 92.0 / 255.0;

        if (node->isGoal)
        {
            m.color.r = 0;
            m.color.g = 1;
            m.color.b = 0;
        }
        else if (node->visited)
        {
            m.color.r = 228.0 / 255.0;
            m.color.g = 87.0 / 255.0;
            m.color.b = 46.0 / 255.0;
        }
        else if (node->isLocal)
        {
            m.color.r = 164.0 / 255.0;
            m.color.g = 145.0 / 255.0;
            m.color.b = 211.0 / 255.0;
        }

        m.pose.position.x = node->p.x();
        m.pose.position.y = node->p.y();

        marker_arr.markers.push_back(m);

        geometry_msgs::Point p1;
        p1.x = node->ps.x();
        p1.y = node->ps.y();
        line_msg.points.push_back(p1);

        geometry_msgs::Point p2;
        p2.x = node->pe.x();
        p2.y = node->pe.y();
        line_msg.points.push_back(p2);

        for (auto child : node->children)
        {

            geometry_msgs::Point tmp1;
            tmp1.x = node->p.x();
            tmp1.y = node->p.y();
            graph_msg.points.push_back(tmp1);

            geometry_msgs::Point tmp2;
            tmp2.x = child->p.x();
            tmp2.y = child->p.y();
            graph_msg.points.push_back(tmp2);
        }

        for (auto parent : node->parents)
        {

            geometry_msgs::Point tmp1;
            tmp1.x = node->p.x();
            tmp1.y = node->p.y();
            graph_msg.points.push_back(tmp1);

            geometry_msgs::Point tmp2;
            tmp2.x = parent->p.x();
            tmp2.y = parent->p.y();
            graph_msg.points.push_back(tmp2);
        }
    }

    marker_arr.markers.push_back(line_msg);
    marker_arr.markers.push_back(graph_msg);
    marker_arr_pub.publish(marker_arr);
    // exit(0);
}

void treeGoalcb(const Eigen::Vector3f &pose)
{

    if (tree->getCount() == 0)
        return;

    float champ_dist = 1000;
    float global_champ_dist = 1000;
    quadtree::Node *champ_node = nullptr;
    quadtree::Node *global_node = nullptr;

    Eigen::Vector2f goal(0, 10);
    Eigen::Vector2f p(pose.x(), pose.y());
    Eigen::Vector4f segment(p.x(), p.y(), goal.x(), goal.y());

    quadtree::Box<float> box(left, top, bwidth, bheight);
    std::vector<quadtree::Node *> nodes = tree->query(box);

    for (auto node : nodes)
    {

        float cost = (node->p - goal).norm(); //*(1+fabs(getAngleFromPoint(pose,node->p)-pose.z()));
        // float cost = (node->p-goal).norm()*(1+fabs(getAngleFromPoint(pose,node->p)-pose.z()));

        Eigen::Vector4f gap(node->ps.x(), node->ps.y(), node->pe.x(), node->pe.y());

        if (node->isGoal)
        {
            cost -= .8;
        }

        if (cost < 0)
            cost = 0;

        // ROS_INFO("Score for (%.2f,%.2f) is: :%.2f", node->p.x(), node->p.y(), cost);

        node->isGoal = false;

        if (!node->visited && cost < champ_dist && isGapAdmissible(node->p, scanData, pose))
        {
            // ROS_INFO("selecting (%.2f, %.2f) as new champ", node->p.x(), node->p.y());

            // if (tmp->isAdmissible){
            champ_dist = cost;
            champ_node = node;
            // }
        }
    }
    // }

    if (champ_node)
    {

        if (!isPointCompletelyFree(champ_node->p, pose, scanData))
        {
            ROS_INFO("----------------------");
            float left = getAngleFromPoint(pose, champ_node->ps);
            float right = getAngleFromPoint(pose, champ_node->pe);
            if (left > right)
            {
                float tmp = right;
                right = left;
                left = tmp;
            }
            std::vector<quadtree::Node *> parents;
            quadtree::Box<float> bbox(
                std::min(pose.x(), champ_node->p.x()), std::min(pose.y(), champ_node->p.y()),
                fabs(pose.x() - champ_node->p.x()), fabs(pose.y() - champ_node->p.y()));
            ROS_INFO("Champ is (%.2f, %.2f) [%.2f, %.2f]",
                     champ_node->p.x(), champ_node->p.y(), left, right);
            ROS_INFO("\tbbox=(%.2f,%.2f,%.2f,%.2f)", bbox.left, bbox.top, bbox.width, bbox.height);
            std::vector<quadtree::Node *> intersecting = tree->query(bbox);

            for (auto node : intersecting)
            {
                float start = getAngleFromPoint(pose, node->ps);
                float end = getAngleFromPoint(pose, node->pe);
                if (start > end)
                {
                    float tmp = start;
                    start = end;
                    end = tmp;
                }
                ROS_INFO("\tintersects with (%.2f, %.2f) [%.2f,%.2f]",
                         node->p.x(), node->p.y(), start, end);
                ROS_INFO("\t\tps = (%.2f, %.2f) ",
                         node->ps.x(), node->ps.y());
                ROS_INFO("\t\tps = (%.2f, %.2f) ",
                         node->pe.x(), node->pe.y());

                if (std::max(left, start) <= std::min(right, end))
                {

                    if (fabs((p - node->p).squaredNorm() - (p - champ_node->p).squaredNorm()) < 1e-2)
                        continue;
                    else if ((p - node->p).squaredNorm() < (p - champ_node->p).squaredNorm())
                    {

                        bool add = true;
                        for (auto n : parents)
                        {
                            if ((n->p - champ_node->p).squaredNorm() < 1e-2)
                            {
                                add = false;
                                break;
                            }
                        }

                        if (add)
                        {
                            ROS_INFO("\tis parent!");
                            parents.push_back(node);
                        }
                    }
                }
            }

            champ_dist = 100000;
            for (auto parent : parents)
            {
                ROS_INFO("looking at goal parents");
                float d = (parent->p - p).squaredNorm();
                if (d < champ_dist /*&& d > JACKAL_WIDTH*JACKAL_WIDTH/6*/
                    && isGapAdmissible(parent->p, scanData, pose) && isGapAdmissible(champ_node->p, parent->p, scanData, pose) && !parent->visited)
                {
                    ROS_INFO("(%.2f, %.2f) is admissible from (%.2f, %.2f)",
                             champ_node->p.x(), champ_node->p.y(), parent->p.x(), parent->p.y());
                    champ_dist = d;
                    champ_node = parent;
                }
                else
                    ROS_INFO("(%.2f, %.2f)\t%d\t%d",
                             parent->p.x(), parent->p.y(),
                             isGapAdmissible(parent->p, scanData, pose),
                             isGapAdmissible(champ_node->p, parent->p, scanData, pose));
            }
        }

        champ_node->isGoal = true;
        currGoal = champ_node;

        // std::vector<Eigen::Vector2f> virtualGap = findVirtualGap(pose, *currGoal, scanData);
        // Eigen::Vector2f mp = (virtualGap[0]+virtualGap[1])/2;
        // ROS_INFO("Intermediate gap is (%.2f, %.2f)", mp.x(), mp.y());
        // ROS_INFO("\tps: (%.2f, %.2f)", virtualGap[0].x(), virtualGap[0].y());
        // ROS_INFO("\tpe: (%.2f, %.2f)", virtualGap[1].x(), virtualGap[1].y());
    }
    else
    {
        // ROS_WARN("No local gap was less than %.2f meters to the goal...", champ_dist);
        currGoal = finalGoal;
        currGoal->isGoal = true;
        ROS_ERROR("No goal exists at all!");
    }

    publishGoal(pose);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rrt_gen");
    ros::NodeHandle nh;

    float goal_x, goal_y;

    nh.getParam("gap_navigation/top", top);
    nh.getParam("gap_navigation/left", left);
    nh.getParam("gap_navigation/bwidth", bwidth);
    nh.getParam("gap_navigation/dist", goalDist);
    nh.getParam("gap_navigation/bheight", bheight);
    nh.getParam("gap_navigation/gapThresh", threshold);

    // std::cout << goal_x << std::endl;
    // std::cout << goal_y << std::endl;
    // exit(0);

    Eigen::Vector3f pose(0, 0, 0);

    quadtree::Box<float> box(left, top, bwidth, bheight);
    tree = new quadtree::Quadtree<quadtree::Node *, decltype(getBox)>(box, getBox);
    narrowTree = new quadtree::Quadtree<quadtree::Node *, decltype(getBox)>(box, getBox);
    finalGoal = new quadtree::Node();

    ros::Subscriber laserSub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, boost::bind(&lasercb, _1, &pose));
    ros::Subscriber odomSub = nh.subscribe<nav_msgs::Odometry>("/odom", 1, boost::bind(&odomcb, _1, &pose));
    ros::Subscriber goalSub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, boost::bind(&goalcb, _1));

    Eigen::Vector2f goal(goal_x, goal_y);
    Eigen::Vector2f goalS(.2 + goal_x, goal_y);
    Eigen::Vector2f goalP(-.2 + goal_x, goal_y);

    // std::cout << goal << std::endl;
    // exit(0);

    finalGoal->p = goal;
    finalGoal->ps = goalS;
    finalGoal->pe = goalP;
    finalGoal->box.left = goal.x() - JACKAL_WIDTH / 2;
    finalGoal->box.top = goal.y() - JACKAL_LENGTH / 2;
    finalGoal->box.width = JACKAL_WIDTH;
    finalGoal->box.height = JACKAL_LENGTH;
    finalGoal->isGoal = false;
    finalGoal->visited = false;
    finalGoal->isLocal = false;
    tree->add(finalGoal);

    marker_arr_pub = nh.advertise<visualization_msgs::MarkerArray>("gaps", 0);
    goal_pub = nh.advertise<std_msgs::Float32MultiArray>("gapGoal", 10);
    currGoal = nullptr;

    ros::spin();

    delete tree;
    delete narrowTree;
}
