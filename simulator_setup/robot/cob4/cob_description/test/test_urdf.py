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


import sys
import os
import unittest
import subprocess

import rosunit

## A sample python unit test
class TestUrdf(unittest.TestCase):

	def test_correct_format(self):
		print(sys.argv)
		print(len(sys.argv))

		if len(sys.argv) < 2:
			self.fail("no urdf file given, usage: " + os.path.basename(sys.argv[0]) + " file.urdf.xacro. \ninput parameters are: " + str(sys.argv))

		file_to_test = sys.argv[1]
		print("testing " + file_to_test)

		# check if file exists
		if not os.path.exists(file_to_test):
			self.fail('file "' + file_to_test + '" not found')

		# check if xacro can be converted
		xacro_args = '--inorder' if (os.environ['ROS_DISTRO'] < 'melodic') else ''
		p = subprocess.Popen("xacro " + xacro_args + " " + file_to_test + " > /tmp/test.urdf", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
		(out,err) = p.communicate()
		if p.returncode != 0 and p.returncode != None:
			self.fail("cannot convert xacro. file: " + file_to_test + "\nOutput: " + out + "\nError: " + err)

		# check if urdf is correct
		p = subprocess.Popen("check_urdf /tmp/test.urdf", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
		if p.returncode != 0 and p.returncode != None:
			self.fail("urdf not correct. file: " + file_to_test + "\n" + p.stderr.read())

if __name__ == '__main__':
	rosunit.unitrun('cob_description', 'test_urdf', TestUrdf)
