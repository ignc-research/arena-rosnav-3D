#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/elias/catkin_ws/src/arena-rosnav-3D/task_generator_gazebo"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/elias/catkin_ws/src/arena-rosnav-3D/install/task_generator_gazebo/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/elias/catkin_ws/src/arena-rosnav-3D/install/task_generator_gazebo/lib/python2.7/dist-packages:/home/elias/catkin_ws/src/arena-rosnav-3D/build/task_generator_gazebo/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/elias/catkin_ws/src/arena-rosnav-3D/build/task_generator_gazebo" \
    "/usr/bin/python2" \
    "/home/elias/catkin_ws/src/arena-rosnav-3D/task_generator_gazebo/setup.py" \
     \
    build --build-base "/home/elias/catkin_ws/src/arena-rosnav-3D/build/task_generator_gazebo" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/elias/catkin_ws/src/arena-rosnav-3D/install/task_generator_gazebo" --install-scripts="/home/elias/catkin_ws/src/arena-rosnav-3D/install/task_generator_gazebo/bin"
