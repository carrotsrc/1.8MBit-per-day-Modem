cmake_minimum_required (VERSION 2.8.11)
project (SideWinder)
set(CMAKE_CXX_STANDARD 14)

set(COMMON_LIBS PUBLIC jack pthread sndfile)

set(COMMON_SRC
        ${CMAKE_CURRENT_SOURCE_DIR}/tokamak/uid.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/tokamak/audio_buffer.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/tokamak/audio_layer.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/tokamak/audio_locus.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/tokamak/audio_signal.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/tokamak/audio_loader.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/tokamak/audio_endpoint.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/tokamak/jack_port_container.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/tokamak/jack_client.cpp
)
add_library( tokamak ${COMMON_SRC} )

add_subdirectory(src)