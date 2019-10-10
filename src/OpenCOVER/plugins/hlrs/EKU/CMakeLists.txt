SET(HEADERS
  EKU.h
  Cam.h
  SafetyZone.h
  openGA.hpp
  GA.hpp
  FileReader.hpp
  Sensor.h
)
SET(SOURCES
  EKU.cpp
  Cam.cpp
  SafetyZone.cpp
  GA.cpp
  FileReader.cpp
  Sensor.cpp
  
)
cover_add_plugin(EKU ${HEADERS} ${SOURCES})
