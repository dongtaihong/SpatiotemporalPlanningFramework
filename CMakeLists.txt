cmake_minimum_required (VERSION 3.2)
project (local_mpc_planner)

set(CMAKE_BUILD_TYPE "Debug")

find_package(Python3 3.8 COMPONENTS Interpreter Development NumPy REQUIRED)

#find_package(Osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories({PYTHON_INCLUDE_DIRS} 
                    include
                    include/dataStructure
                    include/planner
                    include/simulator
                    /usr/local/include/osqp
                    ${EIGEN3_INCLUDE_DIR})
add_executable(${CMAKE_PROJECT_NAME}  
                src/main.cpp
                src/planner/mpcPlanner.cpp
                src/planner/testPlanner.cpp
                src/simulator/randomSimulatorFactory.cpp)

target_link_libraries(${CMAKE_PROJECT_NAME} 
                        Python3::Python
                       Python3::NumPy
                       osqp
                       OsqpEigen::OsqpEigen)