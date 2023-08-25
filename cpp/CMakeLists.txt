cmake_minimum_required(VERSION 3.21.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# PROJECT_VERSION_MAJOR year
# PROJECT_VERSION_MINOR month
# PROJECT_VERSION_PATCH day
# PROJECT_VERSION_TWEAK tweak
project(quadcopter VERSION 2023.01.28.0)

if(PROJECT_IS_TOP_LEVEL)
    cmake_host_system_information(RESULT HOST QUERY HOSTNAME)
    cmake_host_system_information(RESULT OSN QUERY OS_NAME)
    cmake_host_system_information(RESULT OS_VERSION QUERY OS_RELEASE)
    cmake_host_system_information(RESULT PROC QUERY PROCESSOR_DESCRIPTION)

    message(STATUS "-------------------------------------")
    message(STATUS "  Project: ${PROJECT_NAME}")
    message(STATUS "  Version: ${CMAKE_PROJECT_VERSION}")
    message(STATUS "  C++ ${CMAKE_CXX_STANDARD}")
    message(STATUS "-------------------------------------")
    message(STATUS " ${HOST}")
    message(STATUS " ${OSN}: ${OS_VERSION}")
    message(STATUS " ${PROC}")
    message(STATUS "-------------------------------------")

endif()

# Threads -----------------
# find_package(Threads REQUIRED)
# if(Threads_FOUND)
#     message(STATUS ">> Threads found")
#     if(CMAKE_USE_PTHREADS_INIT)
#         message(STATUS " -> Using pthreads")
#     endif()
# else()
#     message(WARNING "*** Could not find Threads ***")
# endif()

include(FetchContent)

# gecko2 -----------------
FetchContent_Declare(gecko2
    GIT_REPOSITORY "https://github.com/gecko-robotics/gecko2.git"
    GIT_TAG "origin/main"
    SOURCE_SUBDIR "cpp"
)

# serialcomm -----------------
FetchContent_Declare(serialcomm
    GIT_REPOSITORY "https://github.com/gecko-robotics/serialcomm.git"
    GIT_TAG "origin/main"
)

# yivo -----------------
FetchContent_Declare(yivo
    GIT_REPOSITORY "https://github.com/MomsFriendlyRobotCompany/yivo.git"
    GIT_TAG "origin/main"
    SOURCE_SUBDIR "c++"
)

# squaternion -----------------
FetchContent_Declare(squaternion
    GIT_REPOSITORY "https://github.com/MomsFriendlyRobotCompany/squaternion.git"
    GIT_TAG "origin/master"
    SOURCE_SUBDIR "cpp"
)

LIST(APPEND libs
    squaternion
    yivo
    # serialcomm
    gecko2
)

FetchContent_MakeAvailable( ${libs} )

# Executables =====================================================================

LIST(APPEND programs
    navigator
    bus
)

foreach(exec ${programs})
    add_executable(${exec} src/${exec}.cpp)
    target_link_libraries(${exec}
        PRIVATE
            ${libs}
    )
endforeach()

# add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
#     COMMAND ${CMAKE_BUILD_DIR}
#     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
#     # COMMENT "Running CRC32 check..."
# )
