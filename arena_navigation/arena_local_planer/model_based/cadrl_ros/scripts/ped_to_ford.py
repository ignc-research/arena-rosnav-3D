#!/usr/bin/env python
from ford_msgs.msg import PedTrajVec, NNActions, PlannerMode, Clusters
from pedsim_msgs.msg import AgentStates
from gazebo_msgs.msg import ModelStates
from obstacle_detector.msg import Obstacles
from geometry_msgs.msg import Vector3
import rospy


class Transformer():
    def __init__(self):
        # models = rospy.wait_for_message("/gazebo/model_states", ModelStates)
        # self.label = 0
        # for i, model_name in enumerate(models.name):
        #     if not (model_name.startswith("ARENA_GEN") or model_name.startswith("person_")):

        self.pedsim_sub = rospy.Subscriber(
            "/pedsim_simulator/simulated_agents", AgentStates, self.transPedMsg)
        self.clusters_pub = rospy.Publisher(
            '/clusters', Clusters, queue_size=1)

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

    def transObstacleMsg(self, msg):
        clusters = Clusters()
        mean_points = []
        velocities = []
        labels = []
        counter = 0
        for circle in msg.circles:
            mean_points.append(circle.center)
            velocities.append(circle.velocity)
            labels.append(counter)
            counter += 1
        for segment in msg.segments:
            # print("FIRST:")
            # print(segment.first_point)
            # print("\n-------------------------\n LAST:")
            # print(segment.last_point)
            # print("\n--------------------\n")
            mean_points.append(segment.last_point)
            labels.append(counter)
            counter += 1
            mean_points.append(segment.first_point)
            labels.append(counter)
            counter += 1
            velocities.append(Vector3(0, 0, 0))
            velocities.append(Vector3(0, 0, 0))

        clusters.mean_points = mean_points
        clusters.velocities = velocities
        clusters.labels = labels
        self.clusters_pub.publish(clusters)


def main():
    rospy.init_node('ped_to_ford', anonymous=True)
    trans = Transformer()
    rospy.spin()


if __name__ == "__main__":
    main()
