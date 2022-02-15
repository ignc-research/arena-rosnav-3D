#!/usr/bin/env python  
# Reads material definitions and publishes visualization message of segment list

import roslib
roslib.load_manifest("light_scan_sim")

import rospy
from light_scan_sim.msg import SegmentList, MaterialList
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Vector3

markers = MarkerArray()
segments = None
materials = None

# Store materials and attempt to generate the marker list
def material_callback(data):
    global materials
    materials = data.materials
    print "Got material list with %d segments" % len(data.materials)
    generate_markers()

# Store segments and attempt to generate the marker list
def segment_callback(data):
    global segments
    segments = data
    print "Got segment list with %d segments" % len(data.segments)
    generate_markers()

# Combine materials and segments into a visualization marker array
def generate_markers():
    global markers, materials, segments

    if materials is None:
        print "Waiting for materials"
        return

    if segments is None:
        print "Waiting on segments"
        return

    markers.markers = []

    for i in range(len(segments.segments)):
      segment = segments.segments[i]

      # Generate marker for each segment
      m = Marker();
      m.header.frame_id = segments.frame_id
      m.header.stamp = rospy.get_rostime()
      m.id = i
      m.type = 4  # Line list
      m.pose.orientation.x = 0
      m.pose.orientation.y = 0
      m.pose.orientation.z = 0
      m.pose.orientation.w = 1.0
      m.scale.x = 0.05
      m.scale.y = 0.05
      m.scale.z = 0.05
      m.action = 0
      m.points.append(Vector3(segment.start[0], segment.start[1], 0))
      m.points.append(Vector3(segment.end[0], segment.end[1], 0))
      if (segment.type < len(materials)):
        m.color.r = materials[segment.type].color[0]
        m.color.g = materials[segment.type].color[1]
        m.color.b = materials[segment.type].color[2]
      m.color.a = 0.8
      markers.markers.append(m)

    print "Generated markers" 

# Publish the 
if __name__ == '__main__':
    rospy.init_node('visualize_segment_list')

    # Todo: Load the material list

    rospy.Subscriber(rospy.get_param('~input_topic', '/segments'), SegmentList, segment_callback)
    rospy.Subscriber(rospy.get_param('~materials_topic', '/materials'), MaterialList, material_callback)
    pub = rospy.Publisher(rospy.get_param('~output_topic', '/segments_vis'), MarkerArray, queue_size=1)

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        # Publish the MarkerArray
        pub.publish(markers)
        rate.sleep()
