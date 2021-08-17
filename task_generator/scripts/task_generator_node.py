#! /usr/bin/env python

import rospy, time
from std_srvs.srv import Empty, EmptyResponse
from nav_msgs.msg import Odometry
from task_generator.tasks import get_predefined_task
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry

class TaskGenerator:
    def __init__(self):
        
        self.sr = rospy.Publisher('/scenario_reset', Int16, queue_size=1)
        self.nr = 0
        mode = rospy.get_param("~task_mode")
        #mode = 'staged'

        scenarios_json_path = rospy.get_param("~scenarios_json_path")
        #scenarios_json_path = '/home/elias/catkin_ws/src/arena-rosnav-3D/simulator_setup/scenarios/test_scenario.json'
        paths = {"scenario": scenarios_json_path}
        # paths = {'curriculum': '/home/elias/catkin_ws/src/arena-rosnav-3D/arena_navigation/arena_local_planer/learning_based/arena_local_planner_drl/configs/training_curriculum_map1small.yaml', 
        #          'robot_setting': '/home/elias/catkin_ws/src/arena-rosnav-3D/simulator_setup/robot/myrobot.model.yaml', 
        #          'robot_as': '/home/elias/catkin_ws/src/arena-rosnav-3D/arena_navigation/arena_local_planer/learning_based/arena_local_planner_drl/configs/default_settings.yaml', 
        #          'hyperparams': '/home/elias/catkin_ws/src/arena-rosnav-3D/arena_navigation/arena_local_planer/learning_based/arena_local_planner_drl/configs/hyperparameters', 
        #          'eval': '/home/elias/catkin_ws/src/arena-rosnav-3D/arena_navigation/arena_local_planer/learning_based/arena_local_planner_drl/training_logs/train_eval_log/MLP_ARENA2D', 
        #          'model': '/home/elias/catkin_ws/src/arena-rosnav-3D/arena_navigation/arena_local_planer/learning_based/arena_local_planner_drl/agents/MLP_ARENA2D', 
        #          'tb': '/home/elias/catkin_ws/src/arena-rosnav-3D/arena_navigation/arena_local_planer/learning_based/arena_local_planner_drl/training_logs/tensorboard/MLP_ARENA2D'}
        self.task = get_predefined_task("",mode, PATHS=paths)

        # if auto_reset is set to true, the task generator will automatically reset the task
        # this can be activated only when the mode set to 'ScenarioTask'
        #auto_reset = True
        auto_reset = rospy.get_param("~auto_reset")
        self.start_time_= time.time()           
        
        # if the distance between the robot and goal_pos is smaller than this value, task will be reset
        self.timeout_= rospy.get_param("~timeout")
        #self.timeout_ = 2.0
        self.timeout_= self.timeout_*60             # sec
        self.start_time_ = time.time()                # sec
        self.delta_ = rospy.get_param("~delta")
        #self.delta_ = 1.0
        robot_odom_topic_name = rospy.get_param(
            "robot_odom_topic_name", "odom")
        
        auto_reset = auto_reset and mode == "scenario"
        self.curr_goal_pos_ = None
        
        
        if auto_reset:
            rospy.loginfo(
                "Task Generator is set to auto_reset mode, Task will be automatically reset as the robot approaching the goal_pos")
            self.reset_task()
            #self.robot_pos_sub_ = rospy.Subscriber(
             #   robot_odom_topic_name, Odometry, self.check_robot_pos_callback)

            rospy.Timer(rospy.Duration(0.5),self.goal_reached)
            
        else:
            # declare new service task_generator, request are handled in callback task generate
            self.reset_task()
                
        self.err_g = 100
        


    def goal_reached(self,event):

        if self.err_g < self.delta_:
            print(self.err_g)
            self.reset_task()
        if(time.time()-self.start_time_>self.timeout_):
            print("timeout")
            self.reset_task()

    def reset_srv_callback(self, req):
        rospy.loginfo("Task Generator received task-reset request!")
        self.task.reset()
        #return EmptyResponse()


    def reset_task(self):
        self.start_time_=time.time()
        info = self.task.reset()
        
        # clear_costmaps()
        if info is not None:
            self.curr_goal_pos_ = info['robot_goal_pos']
        rospy.loginfo("".join(["="]*80))
        rospy.loginfo("goal reached and task reset!")
        rospy.loginfo("".join(["="]*80))
        self.sr.publish(self.nr)
        self.nr += 1
        

    def check_robot_pos_callback(self, odom_msg):
        # type: (Odometry) -> Any
        robot_pos = odom_msg.pose.pose.position
        robot_x = robot_pos.x
        robot_y = robot_pos.y
        goal_x = self.curr_goal_pos_[0]
        goal_y = self.curr_goal_pos_[1]

        self.err_g = (robot_x-goal_x)**2+(robot_y-goal_y)**2
           

if __name__ == '__main__':
    rospy.init_node('task_generator')
    rospy.wait_for_service('/static_map')
    task_generator = TaskGenerator()
    rospy.spin()
