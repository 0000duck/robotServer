include_directories(.)
aux_source_directory(src DIR_ROBOTLINK_SRCS)
add_library(RobotLink ${DIR_ROBOTLINK_SRCS})
