# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "cv_bridge;image_transport;roscpp;sensor_msgs;std_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-llane_detection".split(';') if "-llane_detection" != "" else []
PROJECT_NAME = "lane_detection"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
