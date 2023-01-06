cmake_minimum_required(VERSION 3.5)

project(LelyTest LANGUAGES CXX)
set(EXECUTABLE_NAME ${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(LELY ${CMAKE_CURRENT_SOURCE_DIR}/../3rdParty/lely-core)
# INCLUDE(${LELY}/include-lely-core.cmake)
INCLUDE(${PROJECT_SOURCE_DIR}/../cmake/include-lely-core.cmake)

set(SOURCES
	main.cpp
)

set(HEADERS
)

set (DEST_DIR ${CMAKE_CURRENT_BINARY_DIR})
message(STATUS "CMAKE_CURRENT_BINARY_DIR: ${CMAKE_CURRENT_BINARY_DIR}")

configure_file(motor.dcf ${DEST_DIR}/motor.dcf COPYONLY)
configure_file(motor_4.dcf ${DEST_DIR}/motor_4.dcf COPYONLY)
configure_file(master.dcf ${DEST_DIR}/master.dcf COPYONLY)

set (HARDWARE_VARIANTS "demo")
set (DCF_DEPS)
foreach (hardwareVariant ${HARDWARE_VARIANTS})
	set (input   ${CMAKE_CURRENT_LIST_DIR}/${hardwareVariant}.yml)
	set (destdir ${DEST_DIR}/${hardwareVariant}/)
	set (output  ${destdir}/master.dcf)
	message(STATUS "Generating for hardware variant ${hardwareVariant}: '${input}' --> '${output}'")

	add_custom_command(
		DEPENDS ${input}
		DEPENDS ${CMAKE_CURRENT_LIST_DIR}/demo_motor.eds
		OUTPUT ${output}
		WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
		COMMAND ${CMAKE_COMMAND} -E make_directory ${destdir}
		COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${LELY_DEVENV_LIB}/python3.9/site-packages/ ${LELY_DEVENV_BIN}/dcfgen -v -r -S -d ${destdir} ${input}
	)

	set (DCF_DEPS ${DCF_DEPS} ${output})
endforeach(hardwareVariant)

add_executable(${EXECUTABLE_NAME} ${SOURCES} ${HEADERS} ${DCF_DEPS})

target_include_directories(${EXECUTABLE_NAME}
	PRIVATE ../LelyIntegration/include
	PRIVATE ${LELY_INCLUDE}
)

target_link_libraries(${EXECUTABLE_NAME}
  PRIVATE LelyIntegration
	PRIVATE ${LELY_LIBRARIES}
)

set_target_properties(${EXECUTABLE_NAME} PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

install(TARGETS ${EXECUTABLE_NAME} EXPORT ${PROJECT_NAME} DESTINATION bin)
install(FILES ${DEST_DIR}/motor.dcf DESTINATION etc)
install(FILES ${DEST_DIR}/motor_4.dcf DESTINATION etc)
install(FILES ${DEST_DIR}/master.dcf DESTINATION etc)
install(FILES ${DEST_DIR}/demo/master.dcf DESTINATION etc/demo)
install(FILES ${DEST_DIR}/demo/node_2.bin DESTINATION etc/demo)
install(FILES ${DEST_DIR}/demo/node_3.bin DESTINATION etc/demo)
install(FILES ${DEST_DIR}/demo/node_4.bin DESTINATION etc/demo)
install(FILES ${LELY_LIBRARIES} DESTINATION lib)
