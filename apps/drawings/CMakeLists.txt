add_executable(
    drawings main.cpp app.h app.cpp geometry.h geometry.cpp
    drawing_circle.h drawing_circle.cpp
    drawing_functions.h drawing_functions.cpp
    drawing_polygon.h drawing_polygon.cpp
    imgui_extension.h imgui_extension.cpp
splineio.h splineio.cpp
    straightedge_and_compass_constructions.h straightedge_and_compass_constructions.cpp)

set_target_properties(drawings     PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
target_include_directories(drawings PUBLIC ${CMAKE_SOURCE_DIR}/libs)
target_link_libraries(drawings yocto yocto_gui realtime m)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wno-return-type")
