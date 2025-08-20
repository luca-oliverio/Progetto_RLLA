# CMake generated Testfile for 
# Source directory: /home/leandro/università/Progetto_RLLA
# Build directory: /home/leandro/università/Progetto_RLLA/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
if(CTEST_CONFIGURATION_TYPE MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
  add_test([=[boids_sim.t]=] "/home/leandro/università/Progetto_RLLA/build/Debug/boids_sim.t")
  set_tests_properties([=[boids_sim.t]=] PROPERTIES  _BACKTRACE_TRIPLES "/home/leandro/università/Progetto_RLLA/CMakeLists.txt;53;add_test;/home/leandro/università/Progetto_RLLA/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
  add_test([=[boids_sim.t]=] "/home/leandro/università/Progetto_RLLA/build/Release/boids_sim.t")
  set_tests_properties([=[boids_sim.t]=] PROPERTIES  _BACKTRACE_TRIPLES "/home/leandro/università/Progetto_RLLA/CMakeLists.txt;53;add_test;/home/leandro/università/Progetto_RLLA/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
  add_test([=[boids_sim.t]=] "/home/leandro/università/Progetto_RLLA/build/RelWithDebInfo/boids_sim.t")
  set_tests_properties([=[boids_sim.t]=] PROPERTIES  _BACKTRACE_TRIPLES "/home/leandro/università/Progetto_RLLA/CMakeLists.txt;53;add_test;/home/leandro/università/Progetto_RLLA/CMakeLists.txt;0;")
else()
  add_test([=[boids_sim.t]=] NOT_AVAILABLE)
endif()
