set(LELY "Please Configure")

set (LELY_INCLUDE ${LELY}/include)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
	set (LELY_LIB ${LELY}/dist-arm-debug/lib)
else()
	set (LELY_LIB ${LELY}/dist-arm-release/lib)
endif()
message("Using lely-core libs from " ${LELY_LIB})

set (LELY_LIBRARIES
	${LELY_LIB}/liblely-can.so.1
	${LELY_LIB}/liblely-co.so.2
	${LELY_LIB}/liblely-util.so.2
	${LELY_LIB}/liblely-io2.so.2
	${LELY_LIB}/liblely-coapp.so.2
	${LELY_LIB}/liblely-ev.so.2
	${LELY_LIB}/liblely-libc.so.2
)

set (LELY_DEVENV_ROOT ${LELY}/dist-x86_64-debug)
set (LELY_DEVENV_BIN ${LELY_DEVENV_ROOT}/bin)
set (LELY_DEVENV_LIB ${LELY_DEVENV_ROOT}/lib)
