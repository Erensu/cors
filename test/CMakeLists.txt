if(WIN32)
    set(LIBS winmm)
else()
    set(LIBS sqlite3 m pthread)
endif()

add_executable(test_cors test_cors.c)
target_link_libraries(test_cors cors ${LIBS} uv_a lapack gfortran quadmath)

add_executable(test_bstas_nearest_range test_bstas_nearest_range.c)
target_link_libraries(test_bstas_nearest_range cors ${LIBS} uv_a)

add_executable(test_dtrignet test_dtrignet.c)
target_link_libraries(test_dtrignet cors ${LIBS} uv_a lapack gfortran quadmath)

add_executable(test_rtcm_decoder test_rtcm_decoder.c)
target_link_libraries(test_rtcm_decoder cors ${LIBS} uv_a lapack gfortran quadmath)

add_executable(test_rtkpos test_rtkpos.c)
target_link_libraries(test_rtkpos cors ${LIBS} uv_a lapack gfortran quadmath)

add_executable(test_rtcm_encoder test_rtcm_encoder.c)
target_link_libraries(test_rtcm_encoder cors ${LIBS} uv_a lapack gfortran quadmath)


