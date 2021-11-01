#!/usr/bin/env python
# coding=utf-8

from copy import deepcopy

import rospkg
import rospy
import yaml
from lxml import etree
from lxml.etree import Element
from task_generator.ped_manager.ArenaScenario import \
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
# check = Element("allow_auto_disable")
# check.text = "false"
for model in tree_.xpath("//model[starts-with(@name, 'ARENA_GEN_person_')]"):
    model.getparent().remove(model)
if mode == "scenario":
    scenario_path = rospy.get_param("~scenario_path")
    scenario = ArenaScenario()
    scenario.loadFromFile(scenario_path)
    num_of_actors = 0
    for ped in scenario.pedsimAgents:
        num_of_actors += ped.number_of_peds
else:
    num_of_actors = rospy.get_param("~actors", 3)


for item in range(num_of_actors):
    actor = Element("actor", name="person_"+str(item+1))
    s_pos = etree.fromstring('<pose> -0.46 20.8 0.0 0.0 0.0 1.18 </pose>')
    actor.append(s_pos)
    skin = Element("skin")
    skin_fn = Element("filename")
    skin_fn.text = "model://actor/meshes/SKIN_man_green_shirt.dae"
    skin_scale = Element("scale")
    skin_scale.text = "1"
    skin.append(skin_fn)
    # skin.append(skin_scale)
    actor.append(skin)
    animation = Element("animation", name="animation")
    animate_fn = Element("filename")
    animate_fn.text = "model://actor/meshes/ANIMATION_walking.dae"
    interpolate_x = Element("interpolate_x")
    interpolate_x.text = "true"
    animate_scale = Element("scale")
    animate_scale.text = "1"
    animation.append(animate_fn)
    # animation.append(animate_scale)
    animation.append(interpolate_x)
    actor.append(animation)
    plugin = Element("plugin", name="None", filename='libActorPosePlugin.so')
    model = etree.fromstring(
        f'<model name="ARENA_GEN_person_{str(item+1)}_collision_model"/>')
    # collision_model = etree.fromstring('<plugin name="actor_collisions_plugin" filename="libActorCollisionsPlugin.so"><scaling collision="LHipJoint_LeftUpLeg_collision" scale="         0.01         0.001         0.001       "/><scaling collision="LeftUpLeg_LeftLeg_collision" scale="         8.0         8.0         1.0       "/><scaling collision="LeftLeg_LeftFoot_collision" scale="         8.0         8.0         1.0       "/><scaling collision="LeftFoot_LeftToeBase_collision" scale="         4.0         4.0         1.5       "/><scaling collision="RHipJoint_RightUpLeg_collision" scale="         0.01         0.001         0.001       "/><scaling collision="RightUpLeg_RightLeg_collision" scale="         8.0         8.0         1.0       "/><scaling collision="RightLeg_RightFoot_collision" scale="         8.0         8.0         1.0       "/><scaling collision="RightFoot_RightToeBase_collision" scale="         4.0         4.0         1.5       "/><scaling collision="LowerBack_Spine_collision" scale="         12.0         20.0         5.0       " pose="0.05 0 0 0 -0.2 0"/><scaling collision="Spine_Spine1_collision" scale="         0.01         0.001         0.001       "/><scaling collision="Neck_Neck1_collision" scale="         0.01         0.001         0.001       "/><scaling collision="Neck1_Head_collision" scale="         5.0         5.0         3.0       "/><scaling collision="LeftShoulder_LeftArm_collision" scale="         0.01         0.001         0.001       "/><scaling collision="LeftArm_LeftForeArm_collision" scale="         5.0         5.0         1.0       "/><scaling collision="LeftForeArm_LeftHand_collision" scale="         5.0         5.0         1.0       "/><scaling collision="LeftFingerBase_LeftHandIndex1_collision" scale="         4.0         4.0         3.0       "/><scaling collision="RightShoulder_RightArm_collision" scale="         0.01         0.001         0.001       "/><scaling collision="RightArm_RightForeArm_collision" scale="         5.0         5.0         1.0       "/><scaling collision="RightForeArm_RightHand_collision" scale="         5.0         5.0         1.0       "/><scaling collision="RightFingerBase_RightHandIndex1_collision" scale="         4.0         4.0         3.0       "/></plugin>')
    actor.append(plugin)
    
    coll_plugin = sim_setup_path + '/obstacles/' + 'utils' + '/collision-actor-plugin'
    with open(coll_plugin) as _:
        collision_model = etree.fromstring(_.read())
  
    # collision_model = etree.fromstring(
    #     f'<plugin name="attach_model" filename="libAttachModelPlugin.so">      <link>        <link_name>person_{str(item+1)}_pose</link_name>        <model>          <model_name>ARENA_GEN_person_{str(item+1)}_collision_model</model_name>        </model>      </link>    </plugin>')
   
    actor.append(collision_model)
    world_.append(model)
    world_.append(actor)


tree_.write(world_file,
            pretty_print=True, xml_declaration=True, encoding="utf-8")
