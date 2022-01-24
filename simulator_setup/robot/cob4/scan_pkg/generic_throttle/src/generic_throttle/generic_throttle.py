#!/usr/bin/env python
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rospy
import rostopic
import cv2
from threading import Lock
from threading import currentThread
from functools import partial
from cv_bridge import CvBridge, CvBridgeError


class GenericThrottle:
    def __init__(self):
        self.topics = None
        self.bridge = CvBridge()

        mandatory_parameters = ['topic_rate','latched', 'lazy']

        topics_param_name = '~topics'
        if not rospy.has_param(topics_param_name):
            rospy.logerr('Parameter ' + topics_param_name + ' not available')

        # See README for format
        topics_list = rospy.get_param(topics_param_name, [])  # type: List[Mapping]

        # create dictionary out of the topic list
        self.topics = {next(iter(item.keys())): next(iter(item.values())) for item in topics_list}

        # Check if each entry of topics has the mandatory parameters
        mandatory_flag = all(set(mandatory_parameters) <=
                             set(element) for key, element in
                             self.topics.items())

        if(not(mandatory_flag)):
            rospy.logerr('Each throttled topic needs 3 parameters ' +
                         str(mandatory_parameters))
            exit(10)

        # Populate the dictionary for the ros topic throttling
        self._populate_dictionary()

    def timer_callback(self, event, topic_id):
        # The False argument is for a non blocking call
        locking = self.topics[topic_id]['lock'].acquire_lock(False)

        if not(locking):
            current_t = currentThread()
            rospy.logdebug(str(current_t.getName()) + ': cannot lock topic '
                          + topic_id)
            return

        publisher = self.topics[topic_id]['publisher']
        subscriber = self.topics[topic_id]['subscriber']

        # Check if pub and sub already exist, if not try to create them after
        # checking the ros topic

        if None in(publisher, subscriber):
            topic_info = rostopic.get_topic_class(topic_id)
            if topic_info[0] is None:
                rospy.logwarn('Cannot find topic ' + topic_id)

                self.topics[topic_id]['lock'].release_lock()
                return
            else:
                # Create a (latched if needed) publisher
                self.topics[topic_id]['publisher'] = \
                    rospy.Publisher(topic_id + '_throttled', topic_info[0],
                                    queue_size = 1,
                                    latch = self.topics[topic_id]['latched'])

                rospy.loginfo('Created publisher for ' + topic_id)

                # Create subscriber
                subscriber_partial = partial(self.subscriber_callback,
                                             topic_id=topic_id)
                self.topics[topic_id]['subscriber'] = \
                    rospy.Subscriber(topic_id, topic_info[0],
                                     subscriber_partial)
                rospy.loginfo('Created subscriber for ' + topic_id)

                self.topics[topic_id]['lock'].release_lock()
                return

        last_message = self.topics[topic_id]['last_message']

        if last_message is not None:
            lazy_behavior = \
                self.topics[topic_id]['publisher'].get_num_connections() == 0 \
                and self.topics[topic_id]['lazy']
            if not lazy_behavior:
                # if requested and possible, apply the resolution change
                try:
                    resolution = self.topics[topic_id]['resolution_factor']
                except KeyError:
                    resolution = None

                if resolution is not None:
                    last_message = self._resize_image(last_message, resolution)
                # publish the throttled message
                self.topics[topic_id]['publisher'].publish(last_message)

            self.topics[topic_id]['last_message'] = None
            self.topics[topic_id]['lock'].release_lock()
            return

        self.topics[topic_id]['lock'].release_lock()
        return

    def subscriber_callback(self, data, topic_id):
        locking = self.topics[topic_id]['lock'].acquire_lock(False)
        # The False argument is for a non blocking call

        if not (locking):
            current_t = currentThread()
            rospy.logdebug(str(current_t.getName()) + ': cannot lock topic '
                          + topic_id)
            return

        try:
            resolution = self.topics[topic_id]['resolution_factor']
        except KeyError:
            resolution = None

        if resolution is not None:
            if data._type != 'sensor_msgs/Image':
                rospy.logwarn('resolution_factor option is not available for ' +
                                data._type + '. Topic ' + topic_id +
                                ' will not be resolution throttled.')
                self.topics[topic_id]['resolution_factor'] = None


        self.topics[topic_id]['last_message'] = data
        self.topics[topic_id]['lock'].release_lock()

    def _populate_dictionary(self):
        # Topic dictionary structure
        # {topic_id: {topic_rate, lazy, latched, subscriber,
        # publisher, lock, timer, last_message}

        for key, element in self.topics.items():
            element['lock'] = Lock()
            element['lock'].acquire_lock()
            element['publisher'] = None
            element['subscriber'] = None
            # Create Timer for each topic
            personal_callback = partial(self.timer_callback, topic_id=key)
            element['timer'] = rospy.Timer(
                rospy.Duration(1.0 / element['topic_rate']), personal_callback)
            element['last_message'] = None
            element['lock'].release_lock()

    def _shutdown(self):
        rospy.loginfo('Stopping ' + str(rospy.get_name()))

    def _resize_image(self, data, resolution):
        old_image = None

        try:
            old_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)
            exit(20)

        try:
            if resolution < 1:
                # shrinking
                new_image = cv2.resize(old_image, (0, 0), fx=resolution,
                                       fy=resolution,
                                       interpolation=cv2.INTER_AREA)
            elif resolution > 1:
                # enlarging
                new_image = cv2.resize(old_image, (0, 0), fx=resolution,
                                       fy=resolution)
            else:
                # resolution == 1 --> Don't resize at all...
                new_image = old_image
        except cv2.error as e:
            print(e)
            exit(20)

        try:
            new_message = self.bridge.cv2_to_imgmsg(new_image,
                                                    encoding=data.encoding)
            return new_message
        except CvBridgeError as e:
            print(e)
            exit(20)
