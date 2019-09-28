set(SOURCES main.cpp Config.cpp Application.cpp StaticObstacle.cpp Actor.cpp)

add_executable(sdl-turtle ${SOURCES})
target_link_libraries(sdl-turtle ${SDL2_LIBRARIES} ${SDL2_IMAGE_LIBRARY} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
