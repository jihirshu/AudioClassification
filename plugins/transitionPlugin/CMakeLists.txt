set (TRANSITION_PLUGIN_SRC
     ${CMAKE_CURRENT_LIST_DIR}/AudioClassificationTransitionPlugin.cpp
     ${ROOT_PATH}/plugins/shared/MovoRobotInterface/MovoRobotInterface.cpp
     ${ROOT_PATH}/plugins/shared/MovoRobotInterface/HardwareStartup.cpp
     ${ROOT_PATH}/plugins/shared/MovoRobotInterface/MovoAPI.cpp
     ${ROOT_PATH}/plugins/shared/MovoRobotInterface/MotionPlanner.cpp
     ${ROOT_PATH}/plugins/shared/RRTConnect/RRTConnect.cpp
     ${ROOT_PATH}/plugins/shared/RRTConnect/Tree.cpp)

set (TRANSITION_PLUGIN_PLANNING_SRC
     ${CMAKE_CURRENT_LIST_DIR}/AudioClassificationTransitionPluginPlanning.cpp
     )

ADD_TRANSITION_PLUGIN(audioClassificationTransitionPlugin "${TRANSITION_PLUGIN_SRC}")
target_include_directories(audioClassificationTransitionPlugin PRIVATE ${ROOT_PATH}/plugins/shared/)

ADD_TRANSITION_PLUGIN(audioClassificationTransitionPluginPlanning "${TRANSITION_PLUGIN_PLANNING_SRC}")
target_include_directories(audioClassificationTransitionPluginPlanning PRIVATE ${ROOT_PATH}/plugins/shared/)
