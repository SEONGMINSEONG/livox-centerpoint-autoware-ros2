#!/bin/bash

readonly VERSION_ROS1="ROS1"
readonly VERSION_ROS2="ROS2"
readonly VERSION_HUMBLE="humble"
readonly VERSION_JAZZY="jazzy"

pushd `pwd` > /dev/null
cd `dirname $0`
echo "Working Path: "`pwd`

TARGET_ROS_EDITION=""
TARGET_ROS_DISTRO=""

# Set working ROS version
if [ "$1" = "ROS2" ]; then
    TARGET_ROS_EDITION=${VERSION_ROS2}
elif [ "$1" = "humble" ]; then
    TARGET_ROS_EDITION=${VERSION_ROS2}
    TARGET_ROS_DISTRO=${VERSION_HUMBLE}
elif [ "$1" = "jazzy" ]; then
    TARGET_ROS_EDITION=${VERSION_ROS2}
    TARGET_ROS_DISTRO=${VERSION_JAZZY}
elif [ "$1" = "ROS1" ]; then
    TARGET_ROS_EDITION=${VERSION_ROS1}
else
    echo "Invalid Argument"
    exit
fi
echo "ROS version is: "$TARGET_ROS_EDITION

# Do not use ROS_VERSION as an internal variable: ROS exports ROS_VERSION=1/2
# and colcon uses it to evaluate conditional package.xml dependencies.
if [ "${TARGET_ROS_EDITION}" = "${VERSION_ROS2}" ]; then
    if [ -n "${TARGET_ROS_DISTRO}" ]; then
        if [ -f "/opt/ros/${TARGET_ROS_DISTRO}/setup.bash" ]; then
            source "/opt/ros/${TARGET_ROS_DISTRO}/setup.bash"
        else
            echo "ROS2 distro not found: /opt/ros/${TARGET_ROS_DISTRO}/setup.bash"
            exit 1
        fi
    elif [ "${ROS_VERSION:-}" != "2" ]; then
        echo "ROS2 environment is not sourced. Use './build.sh humble', './build.sh jazzy', or source ROS2 before './build.sh ROS2'."
        exit 1
    fi
fi

# clear `build/` folder.
# TODO: Do not clear these folders, if the last build is based on the same ROS version.
rm -rf ../../build/
rm -rf ../../devel/
rm -rf ../../install/
# clear src/CMakeLists.txt if it exists.
if [ -f ../CMakeLists.txt ]; then
    rm -f ../CMakeLists.txt
fi

# exit

# substitute the files/folders: CMakeList.txt, package.xml(s)
if [ "${TARGET_ROS_EDITION}" = "${VERSION_ROS1}" ]; then
    if [ -f package.xml ]; then
        rm package.xml
    fi
    cp -f package_ROS1.xml package.xml
elif [ "${TARGET_ROS_EDITION}" = "${VERSION_ROS2}" ]; then
    if [ -f package.xml ]; then
        rm package.xml
    fi
    cp -f package_ROS2.xml package.xml
    cp -rf launch_ROS2/ launch/
fi

# build
pushd `pwd` > /dev/null
if [ "${TARGET_ROS_EDITION}" = "${VERSION_ROS1}" ]; then
    cd ../../
    catkin_make -DROS_EDITION=${VERSION_ROS1}
elif [ "${TARGET_ROS_EDITION}" = "${VERSION_ROS2}" ]; then
    cd ../../
    colcon build --packages-select livox_ros_driver2 --cmake-args -DROS_EDITION=${VERSION_ROS2} -DDISTRO_ROS=${TARGET_ROS_DISTRO}
fi
popd > /dev/null

# remove the substituted folders/files
if [ "${TARGET_ROS_EDITION}" = "${VERSION_ROS2}" ]; then
    rm -rf launch/
fi

popd > /dev/null
