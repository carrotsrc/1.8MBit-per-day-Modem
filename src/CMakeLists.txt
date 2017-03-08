set(MOD_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/mod.cpp
)

set(DEM_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/demod.cpp
)

add_executable (mod ${MOD_SRC})
target_include_directories(mod PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../tokamak)
target_link_libraries(mod PUBLIC tokamak ${COMMON_LIBS})

add_executable (demod ${DEM_SRC})
target_include_directories(demod PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../tokamak)
target_link_libraries(demod PUBLIC tokamak fftw3 m ${COMMON_LIBS})

