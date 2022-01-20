#!/usr/bin/env python3

import rospkg
import rospy
from lxml import etree
from lxml.etree import Element
from task_generator.ped_manager.ArenaScenario import ArenaScenario
import numpy as np
import math

START_TIME = 0.0
# time the actor needs change his orientation to face the next waypoint
ORIENTATION_TIME = 1.5
STANDART_ORIENTATION = 0.0
actor_type = 'person'  # [person, box]

debug = False
rospy.init_node("generate_world")
rospack = rospkg.RosPack()
sim_setup_path = rospack.get_path("simulator_setup")

# setting paths
if not debug:
    mode = rospy.get_param("~task_mode", "staged")
    world_name = rospy.get_param("world")
    world_file = (
        sim_setup_path + "/worlds/" + world_name + "/worlds/" + world_name + ".world"
    )

else:
    mode = "scenario"
    world_file = '/home/elias/catkin_ws/src/arena-rosnav-3D/simulator_setup/worlds/turtlebot3_world/worlds/turtlebot3_world.world'


# extract data
tree_ = etree.parse(world_file)
world_ = tree_.getroot().getchildren()[0]

# removing old actor elements from .world file, before creating new onces
for actor in tree_.xpath("//actor"):
    actor.getparent().remove(actor)


def create_trajectory_element(time, pose):
    """creating the element in the sdf actor-syntax
    """
    waypoint = Element("waypoint")
    t = etree.fromstring(f"<time>{time}</time>")
    waypoint.append(t)
    pose = etree.fromstring(f"<pose> {pose} </pose>")
    waypoint.append(pose)
    trajectory.append(waypoint)


def calculate_trajectory_time(pos_0, pos_1, max_vel):
    """we need to calculate at which time the actor has to be at the next waypoint (in which specific orientation)
    """
    dist = np.linalg.norm(pos_1 - pos_0)
    return dist / max_vel


def create_trajectory_from_two_waypoints(start_point, end_point, traj_time, max_vel, yaw_old):
    """The scenario file defines waypoints [x,y], gazebo however needs trajectory points [x y z roll pitch yaw]
    Args:
        start_point (list): first trajectory point
        end_point (list): second trajectory point
    """
    if not start_point[0] == end_point[0] or not start_point[1] == end_point[1]:
        time = calculate_trajectory_time(start_point, end_point, max_vel)
        if time > ORIENTATION_TIME:

            # orientation to the first waypoint
            yaw = math.atan2(end_point[1]-start_point[1],
                             end_point[0]-start_point[0])
            traj_time += ORIENTATION_TIME
            if not yaw-yaw_old == 0.0:
                pose = f"{start_point[0]} {start_point[1]}  0.0 0.0 0.0 {yaw}"
                create_trajectory_element(traj_time, pose)

            traj_time += time - ORIENTATION_TIME
            # movement to the first waypoint
            pose = f"{end_point[0]} {end_point[1]} 0.0 0.0 0.0 {yaw}"
            create_trajectory_element(traj_time, pose)

            yaw_old = yaw
        else:
            raise NotImplementedError
    else:
        return traj_time, yaw_old

    return traj_time, yaw_old


def add_actor_element_person(j, agent):
    actor = Element("actor", name="person_" +
                    str(j) + "_" + str(agent))
    skin = Element("skin")

    # setting start position
    init_pose = etree.fromstring(
        f"<pose> {ped.pos[0]} {ped.pos[1]} 0.0 0.0 0.0 0.0 </pose>"
    )
    actor.append(init_pose)

    # adding standard actor elements
    skin_fn = Element("filename")
    skin_fn.text = "model://actor/meshes/SKIN_man_green_shirt.dae"
    skin.append(skin_fn)
    actor.append(skin)
    animation = Element("animation", name="animation")
    animate_fn = Element("filename")
    animate_fn.text = "model://actor/meshes/ANIMATION_walking.dae"
    interpolate_x = Element("interpolate_x")
    interpolate_x.text = "true"
    animate_scale = Element("scale")
    animate_scale.text = "1"
    animation.append(animate_fn)
    animation.append(interpolate_x)
    actor.append(animation)
    script = Element("script")
    return actor, script


def add_actor_element_box(j, agent):
    # creating the elements
    actor = Element("actor", name="box_" + str(j) + "_" + str(agent))
    link = Element("link", name="link")
    visual = Element("visual", name="visual")
    geometry = Element("geometry")
    box = Element("box")
    size = Element("size")
    size.text = ".2 .2 .2"

    script = Element("script")
    loop = Element("loop")
    loop.text = "true"
    delay_start = Element("delay_start")
    delay_start.text = "0.000000"
    auto_start = Element("auto_start")
    auto_start.text = "true"

    # appending the created elements
    box.append(size)
    geometry.append(box)
    visual.append(geometry)
    link.append(visual)
    actor.append(link)
    script.append(loop)
    script.append(delay_start)
    script.append(auto_start)

    return actor, script


# In the scenario case we use the gazebo-actor concept to move our dynamic obstacles, therefore we need to script the actors trajectories in the .wold file
if mode in ["scenario", "scenario_staged"]:

    # loading scenario data
    if not debug:
        scenario_path = rospy.get_param("~scenario_path")
    else:
        scenario_path = '/home/elias/catkin_ws/src/arena-rosnav-3D/simulator_setup/scenarios/turtlebot3_world.json'
    scenario = ArenaScenario()
    scenario.loadFromFile(scenario_path)
    num_of_actors = 0

    # looping through the data of every individual dynamic obstacle defined in the scenario file
    for j, ped in enumerate(scenario.pedsimAgents):
        for agent in range(ped.number_of_peds):

            # NOTE: check out 'simulator_setup/worlds/small_warehouse/models/actor/meshes' for further actor models
            if actor_type == 'person':
                actor, script = add_actor_element_person(j, agent)
            if actor_type == 'box':
                actor, script = add_actor_element_box(j, agent)

            # creating the the trajectory points
            trajectory = Element(
                "trajectory", id=f"{j}_{agent}", type="animation")
            waypoint = Element("waypoint")
            max_vel = ped.vmax
            traj_time = START_TIME
            yaw_old = STANDART_ORIENTATION
            list_of_waypoints = [*ped.waypoints, ped.waypoints[0]]

            # creating orientation & distance waypoint in the actor format
            for pos_1, pos_2 in zip(list_of_waypoints, list_of_waypoints[1:]):
                traj_time, yaw_old = create_trajectory_from_two_waypoints(
                    pos_1, pos_2, traj_time, max_vel, yaw_old)

            script.append(trajectory)

            if actor_type == 'person':
                coll_plugin = (
                    sim_setup_path + "/obstacles/" + "utils" + "/collision-actor-plugin"
                )
                print('IMPORTANT:', coll_plugin)
                with open(coll_plugin) as _:
                    collision_model = etree.fromstring(_.read())
                actor.append(collision_model)

            actor.append(script)
            world_.append(actor)

# in the case of for example random-mode the trajectory points of dynamic obstacles are determined randomly in the task-generator node. The position of the actors is determined by pedsim. Therefore we include here the actor model without trajectory points
else:
    num_of_actors = rospy.get_param("actors", 3)

    for item in range(num_of_actors):
        actor = Element("actor", name="person_" + str(item + 1))
        s_pos = etree.fromstring("<pose> -0.46 20.8 0.0 0.0 0.0 1.18 </pose>")
        actor.append(s_pos)
        skin = Element("skin")
        skin_fn = Element("filename")
        skin_fn.text = "model://actor/meshes/SKIN_man_green_shirt.dae"
        skin_scale = Element("scale")
        skin_scale.text = "1"
        skin.append(skin_fn)
        actor.append(skin)
        animation = Element("animation", name="animation")
        animate_fn = Element("filename")
        animate_fn.text = "model://actor/meshes/ANIMATION_walking.dae"
        interpolate_x = Element("interpolate_x")
        interpolate_x.text = "true"
        animate_scale = Element("scale")
        animate_scale.text = "1"
        animation.append(animate_fn)
        animation.append(interpolate_x)
        actor.append(animation)
        plugin = Element("plugin", name="None",
                         filename="libActorPosePlugin.so")
        actor.append(plugin)

        coll_plugin = (
            sim_setup_path + "/obstacles/" + "utils" + "/collision-actor-plugin"
        )
        with open(coll_plugin) as _:
            collision_model = etree.fromstring(_.read())

        actor.append(collision_model)

        world_.append(actor)


tree_.write(world_file, pretty_print=True,
            xml_declaration=True, encoding="utf-8")
