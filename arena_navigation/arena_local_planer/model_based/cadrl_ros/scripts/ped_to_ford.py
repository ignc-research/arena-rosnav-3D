#!/usr/bin/env python3
import rospy
from ford_msgs.msg import Clusters, NNActions, PedTrajVec, PlannerMode
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Vector3
from pedsim_msgs.msg import AgentStates


class Transformer:
    def __init__(self):

        self.task_mode = rospy.get_param("task_mode", "scenario")
        # Check if using scripted trajectories or pedsim
        if self.task_mode != "scenario":
            self.pedsim_sub = rospy.Subscriber(
                "/pedsim_simulator/simulated_agents", AgentStates, self.transPedMsg
            )
        else:
            self.actors_sub = rospy.Subscriber(
                "/gazebo/model_states", ModelStates, self.transGazToCluster
            )
        self.clusters_pub = rospy.Publisher("/clusters", Clusters, queue_size=1)

    def transPedMsg(self, msg):
        clusters = Clusters()
        mean_points = []
        velocities = []
        labels = []
        for actor in msg.agent_states:
            labels.append(actor.id)
            mean_points.append(actor.pose.position)
            velocities.append(actor.twist.linear)
        clusters.mean_points = mean_points
        clusters.velocities = velocities
        clusters.labels = labels
        self.clusters_pub.publish(clusters)

    def transGazToCluster(self, msg):
        clusters = Clusters()
        mean_points = []
        velocities = []
        labels = []
        actors = [
            msg.name.index(name) for name in msg.name if name.startswith("person_")
        ]
        for index in actors:
            mean_points.append(msg.pose[index].position)
            velocities.append(msg.twist[index].linear)
            labels.append(index)
        clusters.mean_points = mean_points
        clusters.velocities = velocities
        clusters.labels = labels
        self.clusters_pub.publish(clusters)


def main():
    rospy.init_node("ped_to_ford", anonymous=True)
    trans = Transformer()
    rospy.spin()


if __name__ == "__main__":
    main()
