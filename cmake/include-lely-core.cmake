set(LELY /usr/local)
set(LELY_INCLUDE ${LELY}/include)
set(LELY_LIB /usr/local/lib)

# if (CMAKE_BUILD_TYPE STREQUAL "Debug")
# 	set (LELY_LIB ${LELY}/dist-arm-debug/lib)
# else()
# 	set (LELY_LIB ${LELY}/dist-arm-release/lib)
# endif()
message("Using lely-core libs from " ${LELY_LIB})

set (LELY_LIBRARIES
	${LELY_LIB}/liblely-libc.so
	${LELY_LIB}/liblely-tap.so
	${LELY_LIB}/liblely-util.so
	${LELY_LIB}/liblely-can.so
	${LELY_LIB}/liblely-co.so
	${LELY_LIB}/liblely-ev.so
	${LELY_LIB}/liblely-io2.so
	${LELY_LIB}/liblely-coapp.so
)

set (LELY_DEVENV_ROOT ${LELY})
set (LELY_DEVENV_BIN ${LELY_DEVENV_ROOT}/bin)
set (LELY_DEVENV_LIB ${LELY_DEVENV_ROOT}/lib)
