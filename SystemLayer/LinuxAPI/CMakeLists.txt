find_package(Threads)

aux_source_directory(src DIR_LINUXAPI_SRCS)

add_library(SystemAPI ${DIR_LINUXAPI_SRCS})
target_link_libraries(SystemAPI ${CMAKE_THREAD_LIBS_INIT})
