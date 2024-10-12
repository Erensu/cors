cmake_minimum_required(VERSION 3.16)
project(cors)

add_subdirectory(lib/libuv)
add_subdirectory(lib/lapack)

include_directories(${PROJECT_BINARY_DIR})
include_directories(include)
include_directories(lib/libuv)
include_directories(lib/uthash)
include_directories(lib/queue)
include_directories(lib/libuv/include)
include_directories(lib/triangulator)
include_directories(lib/kdtree)

aux_source_directory(src/gpos SRC_GPOS)
aux_source_directory(src/rtcm SRC_RTCM)
aux_source_directory(src/ntrip SRC_NTRIP)
aux_source_directory(src/common SRC_COMMON)
aux_source_directory(src/cors SRC_CORS)
aux_source_directory(src/pnt SRC_PNT)
aux_source_directory(src/nrtk SRC_NRTK)
aux_source_directory(src/monitor SRC_MONITOR)
aux_source_directory(src/dtrignet SRC_DELTRIG)
aux_source_directory(src/vrs SRC_VRS)
aux_source_directory(src/rinex SRC_RINEX)
aux_source_directory(lib/triangulator SRC_TRIANGULATOR)
aux_source_directory(lib/kdtree SRC_KDTREE)

if(WIN32)
    set(LIBS Winmm)
else()
    set(LIBS sqlite3 m pthread)
endif()

add_library(cors ${SRC_GPOS} ${SRC_RTCM} ${SRC_NTRIP} ${SRC_COMMON} ${SRC_CORS} ${SRC_PNT}
        ${SRC_MONITOR} ${SRC_NRTK}
        ${SRC_TRIANGULATOR} ${SRC_KDTREE}
        ${SRC_DELTRIG} ${SRC_VRS} ${SRC_RINEX})
target_link_libraries(cors ${LIBS} lapack gfortran quadmath)

add_executable(cors-engine src/engine/engine.c)
target_link_libraries(cors-engine cors uv)

add_executable(cors-mengine src/engine/mengine.c)
target_link_libraries(cors-mengine cors uv)

add_subdirectory(test)







