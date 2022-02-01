^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pmb2_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.20 (2021-09-01)
-------------------
* Merge branch 'gallium_fixes' into 'erbium-devel'
  Gallium fixes
  See merge request robots/pmb2_robot!73
* Using xacro instead of deprecated xacro.py
* Contributors: Jordan Palacios

3.0.19 (2021-08-11)
-------------------
* Merge branch 'feature-use-master-calibration-for-rgbd-cameras' into 'erbium-devel'
  Load rgbd cameras extrinsic parameters from master calibration
  See merge request robots/pmb2_robot!71
* removed include file duplicated and added comment
* added rgbd calibration feature for courier
* Contributors: josegarcia, victor

3.0.18 (2021-07-19)
-------------------
* Merge branch 'expand-world-odometry' into 'erbium-devel'
  use our world odometry plugin
  See merge request robots/pmb2_robot!68
* Merge branch 'update-apache-license' into 'erbium-devel'
  Update apache license
  See merge request robots/pmb2_robot!69
* Update apache license
* use our world odometry plugin
* Contributors: ThomasPeyrucain, Victor Lopez, victor

3.0.17 (2021-06-23)
-------------------

3.0.16 (2021-02-15)
-------------------
* Merge branch 'bigger-collision-mesh' into 'erbium-devel'
  Use bigger mesh for collisions
  See merge request robots/pmb2_robot!66
* Use bigger mesh for collisions
* Contributors: victor

3.0.15 (2021-01-28)
-------------------

3.0.14 (2021-01-18)
-------------------
* Merge branch 'fix_wheel_slippage' into 'erbium-devel'
  Fix wheel slippage
  See merge request robots/pmb2_robot!62
* Tuning mu1,mu2 parameters for reducing slippage during pure rotational speed cmds
* Uss sphere and tuned contacts for collision links
* test sphere collision mesh and contact parameters
* Contributors: Luca Marchionni, victor

3.0.13 (2020-07-30)
-------------------

3.0.12 (2020-07-16)
-------------------

3.0.11 (2020-07-10)
-------------------
* Merge branch 'elp-camera' into 'erbium-devel'
  Fix ELP rgb camera position and add its gazebo plugin
  See merge request robots/pmb2_robot!58
* Fix ELP rgb camera position and add its gazebo plugin
* Contributors: Sara Cooper, procopiostein

3.0.10 (2019-10-21)
-------------------

3.0.9 (2019-10-02)
------------------

3.0.8 (2019-09-27)
------------------

3.0.7 (2019-09-25)
------------------

3.0.6 (2019-09-20)
------------------
* scan_raw is the default laser topic
* Contributors: Procópio Stein

3.0.5 (2019-09-10)
------------------
* Melodic compatibility
* Contributors: Victor Lopez

3.0.4 (2019-07-17)
------------------
* Merge branch 'multi_pmb2' into 'erbium-devel'
  Changes for multi pmb2 simulation
  See merge request robots/pmb2_robot!44
* Changes for multi pmb2 simulation
* Contributors: Adria Roig, Victor Lopez

3.0.3 (2019-04-09)
------------------
* Merge branch 'enable_sonars' into 'erbium-devel'
  Add sonars argument to base_sensors
  See merge request robots/pmb2_robot!42
* Added sonars argument to base_sensors
* Contributors: Jordan Palacios, Victor Lopez

3.0.2 (2019-01-31)
------------------
* Merge branch 'fix-inertia' into 'erbium-devel'
  Fix inertial parameters of the caster wheels
  See merge request robots/pmb2_robot!41
* Fix inertial parameters of the caster wheels
  Also added friction and damping to improve behavior
* Contributors: Victor Lopez

3.0.1 (2018-12-20)
------------------
* Fix tests
* Contributors: Victor Lopez

3.0.0 (2018-12-19)
------------------
* Merge branch 'specifics-refactor' into 'erbium-devel'
  Remove upload_pmb2.launch
  See merge request robots/pmb2_robot!40
* Add rgbd sensors
* Change robot parameter name
* Parametrize urdf
* Remove upload_pmb2.launch
* Contributors: Victor Lopez

2.0.8 (2018-11-27)
------------------
* Merge branch 'remove-caster-friction' into 'erbium-devel'
  Remove caster friction so it doesn't push base around
  See merge request robots/pmb2_robot!34
* Remove caster friction so it doesn't push base around
* Contributors: Victor Lopez

2.0.7 (2018-07-30)
------------------
* Merge branch 'fix-xacro-warnings' into 'erbium-devel'
  prepend missing 'xacro' tag
  See merge request robots/pmb2_robot!33
* prepend missing 'xacro' tag
* Merge branch 'fix-warning-typo' into 'erbium-devel'
  fix typo
  See merge request robots/pmb2_robot!32
* fix typo
* Contributors: Hilario Tome, Jordi Pages, Victor Lopez

2.0.6 (2018-04-27)
------------------
* Merge branch 'fix_tf_depth_sensor' into 'erbium-devel'
  fixed the frame wrongly removed previously
  See merge request robots/pmb2_robot!31
* removed rgb frames that are not present in this sensor
* fixed the frame wrongly removed previously
* Contributors: Andrei Pasnicenco, Hilario Tome, Procópio Stein

2.0.5 (2018-04-17)
------------------
* Merge branch 'fix-tests-broken-due-to-stl' into 'erbium-devel'
  Revert "fixed warning when loading stl file"
  See merge request robots/pmb2_robot!29
* Revert "fixed warning when loading stl file"
  This reverts commit 49e84804a24372815b2b500159369f1d63d02857.
* Contributors: Hilario Tome, Procópio Stein

2.0.4 (2018-04-17)
------------------

2.0.3 (2018-04-17)
------------------
* Merge branch 'test-branch' into 'erbium-devel'
  Test branch
  See merge request robots/pmb2_robot!27
* Merge branch 'fix-stl' into test-branch
* Merge remote-tracking branch 'origin/fix_xacro_warning' into test-branch
* fixed warning when loading stl file
* fix missing xacro namespace
* Merge remote-tracking branch 'origin/fixing_sim' into test-branch
* Merge remote-tracking branch 'origin/deprecate_upload_pmb2' into test-branch
* Merge remote-tracking branch 'origin/fix_xacro_warning' into test-branch
* updated urdf file to get correct mesh and remove rgb related info
* added structure sensor mesh
* deprecate upload_pmb2
* normalize xmlns across xacro files
* fix xacro warning
  deprecated: xacro tags should be prepended with 'xacro' xml namespace.
  Use the following script to fix incorrect usage:
  find . -iname "*.xacro" | xargs sed -i 's#<\([/]\?\)\(if\|unless\|include\|arg\|property\|macro\|insert_block\)#<\1xacro:\2#g'
* rm usuless caster 1 collision mesh
* fix casters
* Contributors: Jeremie Deray, Procópio Stein

2.0.2 (2018-04-13)
------------------
* reduced sonars max range to avoid noise
* Contributors: Procópio Stein

2.0.1 (2018-03-29)
------------------
* delete transmission for passive joints
* Contributors: Andrei Pasnicenco

2.0.0 (2018-03-26)
------------------
* Merge branch 'collision_fix' into 'erbium-devel'
  caster wheels and base collision
  See merge request robots/pmb2_robot!19
* gtests passed on the flat surface
* linear move test passed
* revert testing torque value
* transmission caster
* sufficient for tests
* fix castor joints transmission
* rotate and climb with a virtual suspension system and use caster wheels
* virtual suspension and rolling caster wheels
* high-res mesh of the base for visual geometry
* Friction coeffs adjusted
* caster wheels and base collision
  Note: Frictin coeffs mu1, mu2 in caster.gazebo.xacro can me adjusted to make robot stopping immideately
* Contributors: Andrei Pasnicenco, Victor Lopez

1.1.14 (2018-01-30)
-------------------
* Merge branch 'test_urdf' into 'dubnium-devel'
  Add URDF tests
  See merge request robots/pmb2_robot!17
* Remove exec dependencies for pal_gazebo_plugins
* Add URDF tests
* Contributors: Adria Roig, Adrià Roig, davidfernandez

1.1.13 (2017-09-27)
-------------------
* renamed depth sensor
* add rgbd related files and config to description
* Contributors: Procópio Stein

1.1.12 (2017-06-30)
-------------------

1.1.11 (2017-06-30)
-------------------

1.1.10 (2017-06-29)
-------------------

1.1.9 (2017-06-28)
------------------
* upgraded packages format, maintainers and license
* Contributors: Procópio Stein

1.1.8 (2017-04-11)
------------------

1.1.7 (2017-02-23)
------------------

1.1.6 (2016-11-07)
------------------
* invert sonars 1 and 3
* Contributors: Jordi Pages

1.1.5 (2016-10-24)
------------------
* Now launch files are more like those for TIAGo
* add tiago_support as maintainer
* Contributors: Jordan Palacios, Jordi Pages

1.1.4 (2016-07-04)
------------------
* corrected imu frame, z always point upwards
  this is because the imu 6050 zeros itself (at least wrt pitch)
* Contributors: Procópio Stein

1.1.3 (2016-06-15)
------------------
* update sonars min/max range
* Contributors: Jeremie Deray

1.1.2 (2016-06-03)
------------------
* sonar ID two digit
* Add imu controller to launch
* Add imu gazebo plugin config
* 1.1.1
* Update changelog
* Updated to new generic pal hardware gazebo plugin
* Simplified base collision
  Now the base_link has a mesh that touches with the ground
* Contributors: Jeremie Deray, Sam Pfeiffer

1.1.1 (2016-04-15)
------------------
* Updated to new generic pal hardware gazebo plugin
* Simplified base collision
  Now the base_link has a mesh that touches with the ground
* Contributors: Sam Pfeiffer

1.1.0 (2016-03-15)
------------------
* urdf use macro param default value
* fix urdf laser
* Contributors: Jeremie Deray

1.0.6 (2016-03-03)
------------------

1.0.5 (2016-02-09)
------------------
* update gazebo sick 561 571 with proper params
* rename base_default to base_sensors
* remove base_full.urdf.xacro
* add gazebo draft sick 561 & 571
* pmb2 desscription upload default
* rm full urdf
* base_default now holds all sensors with option
* pmb2 urdf diff Sick
* Contributors: Jeremie Deray

1.0.4 (2015-10-26)
------------------

1.0.3 (2015-10-06)
------------------

1.0.2 (2015-10-05)
------------------

1.0.1 (2015-10-01)
------------------
* 1.0.0
* Add changelog
* Add changelog
* Merging metal base branch
* urdf full calls default & add sonar/micro
* urdf default calls base & add laser
* urdf base contains basics e.g. wheels
* add full_sick urdf
* add base_default urdf
* renamed base -> base_full
* Update maintainer
* Replace caster collision with spheres, fix spinning
* Remove spanish character nonvalid to xacro
* Update placement and name of base imu
* Add collision to antenna
* Update caster locations
* Add microphone locations
* Added sonars with proper colors
* Add color to gazebo
* Add antennas
* New meshes
* Remove references to xtion
* Remove robot model scripts
* Add inertial params to main body
* Remove bumpers
* Remove rear cover
* More battery removed
* Remove charger
* Remove battery
* Remove base_rgbd
* Fix color of wheels in gazebo
* Add new cover and orange ring around body
* Contributors: Bence Magyar, Jeremie Deray, Luca Marchionni

1.0.0 (2015-09-29)
------------------
* Add changelog
* Merging metal base branch
* urdf full calls default & add sonar/micro
* urdf default calls base & add laser
* urdf base contains basics e.g. wheels
* add full_sick urdf
* add base_default urdf
* renamed base -> base_full
* Update maintainer
* Replace caster collision with spheres, fix spinning
* Remove spanish character nonvalid to xacro
* Update placement and name of base imu
* Add collision to antenna
* Update caster locations
* Add microphone locations
* Added sonars with proper colors
* Add color to gazebo
* Add antennas
* New meshes
* Remove references to xtion
* Remove robot model scripts
* Add inertial params to main body
* Remove bumpers
* Remove rear cover
* More battery removed
* Remove charger
* Remove battery
* Remove base_rgbd
* Fix color of wheels in gazebo
* Add new cover and orange ring around body
* Contributors: Bence Magyar, Jeremie Deray, Luca Marchionni

0.10.0 (2015-07-14)
-------------------

0.9.10 (2015-02-27)
-------------------
* Merge from REEM-C params
* Fix and add link names in macro
* Contributors: Bence Magyar

0.9.9 (2015-02-18)
------------------

0.9.8 (2015-02-18)
------------------
* Add inertial block to xtion pro live
* Add inertial block to range sensor
* Add conditional for base rgbd sensor
* Chop off frontal antennas
* Use ${name} for imu
* Put sonars with its rear cover
* Make rgbd camera fixed
* Add microphones
* Add bumper
* Update meshes
* Use base_footprint_link
* Update meshes
* Add comment to show Joint, Child, Parent
* Remove sensors not needed
* Use 0.27m for footprint radius
* Add kinematics and stl files (except for the base)
* Add kinematics xlsx to URDF converter/helper
* Contributors: Bence Magyar, Enrique Fernandez

0.9.7 (2015-02-02)
------------------
* Update URDF (only locations)
* Replace ant -> pmb2
* Rename files
* Contributors: Enrique Fernandez
