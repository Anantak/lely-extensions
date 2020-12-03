project("LelyIntegration")

set(HEADERS
  ./include/DCFConfigMaster.h
  ./include/DCFDriverConfig.h
  ./include/DCFDriver.h
  ./include/MotorDriver.h
)

set(SOURCES
  ./src/DCFConfigMaster.cpp
  ./src/DCFDriverConfig.cpp
  ./src/DCFDriver.cpp
  ./src/MotorDriver.cpp
)

set(LELY ${CMAKE_CURRENT_SOURCE_DIR}/../3rdParty/lely-core)
INCLUDE(${LELY}/include-lely-core.cmake)

add_library(LelyIntegration STATIC
  ${SOURCES}
  ${HEADERS}
)
target_include_directories(LelyIntegration
  PRIVATE ./include
  PRIVATE ${LELY_INCLUDE}
)
