find_package(catkin REQUIRED roscpp ObservationService)

set(include_dirs
    ${catkin_INCLUDE_DIRS}
    ${ROOT_PATH}/plugins/shared/)

ADD_OBSERVATION_PLUGIN(audioClassificationObservationPlugin ${CMAKE_CURRENT_LIST_DIR}/AudioClassificationObservationPlugin.cpp)
ADD_OBSERVATION_PLUGIN(audioClassificationObservationPluginExecution ${CMAKE_CURRENT_LIST_DIR}/AudioClassificationObservationPluginExecution.cpp)
target_include_directories(audioClassificationObservationPluginExecution PRIVATE "${include_dirs}")
