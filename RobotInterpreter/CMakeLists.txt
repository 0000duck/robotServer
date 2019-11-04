add_definitions("-DDLL_EXPORT")
include_directories(. ../RobotStructure)

aux_source_directory(Common DIR_ROBOTINTERPRETER_SRCS)
aux_source_directory(src DIR_ROBOTINTERPRETER_SRCS)
add_library(RobotInterpreter ${DIR_ROBOTINTERPRETER_SRCS})
