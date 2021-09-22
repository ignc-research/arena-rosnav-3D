#!/usr/bin/env python
# coding=utf-8


import rospkg
import rospy
import yaml
from lxml import etree
from lxml.etree import Element
from task_generator.task_generator.ped_manager.ArenaScenario import \
    ArenaScenario

rospack = rospkg.RosPack()

rospy.init_node('generate_world')

sim_setup_path = rospack.get_path("simulator_setup")
mode = rospy.get_param("~task_mode")
world_name = rospy.get_param("world")
world_file = sim_setup_path+'/worlds/' + \
    world_name + '/worlds/' + world_name + '.world'
tree_ = etree.parse(world_file)
world_ = tree_.getroot().getchildren()[0]
for actor in tree_.xpath("//actor"):
    actor.getparent().remove(actor)

if mode == "random":
    num_of_actors = rospy.get_param("~actors", 3)
else:
    scenario_path = rospy.get_param("~scenario_path")
    scenario = ArenaScenario()
    scenario.loadFromFile(scenario_path)
    num_of_actors = 0
    for ped in scenario.pedsimAgents:
        num_of_actors += ped.number_of_peds

for item in range(num_of_actors):
    actor = Element("actor", name="person_"+str(item+1))
    skin = Element("skin")
    skin_fn = Element("filename")
    skin_fn.text = "model://actor/meshes/SKIN_man_green_shirt.dae"
    skin_scale = Element("scale")
    skin_scale.text = "1"
    skin.append(skin_fn)
    skin.append(skin_scale)
    actor.append(skin)
    animation = Element("animation", name="animation")
    animate_fn = Element("filename")
    animate_fn.text = "model://actor/meshes/ANIMATION_walking.dae"
    interpolate_x = Element("interpolate_x")
    interpolate_x.text = "true"
    animate_scale = Element("scale")
    animate_scale.text = "1"
    animation.append(animate_fn)
    animation.append(animate_scale)
    animation.append(interpolate_x)
    actor.append(animation)
    plugin = Element("plugin", name="None", filename='libActorTestPlugin.so')
    actor.append(plugin)

    world_.append(actor)

tree_.write(world_file,
            pretty_print=True, xml_declaration=True, encoding="utf-8")
