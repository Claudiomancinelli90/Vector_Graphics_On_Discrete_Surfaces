add_subdirectory(yocto)
include_directories(nanosvg/src)
add_subdirectory(realtime)

if(YOCTO_OPENGL)
add_subdirectory(yocto_gui)
endif(YOCTO_OPENGL)
