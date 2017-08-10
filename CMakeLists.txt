cmake_minimum_required(VERSION 3.8)
project(physxOpenGL)

set(CMAKE_CXX_STANDARD 11)

# OpenGL
FIND_PACKAGE(OpenGL REQUIRED)
LIST(APPEND LIBS ${OPENGL_gl_LIBRARY})
LIST(APPEND LIBS ${OPENGL_glu_LIBRARY})

# freeglut
include_directories(3rd_party/freeglut/include)
#LIST(APPEND LIBS freeglut)
LIST(APPEND LIBS glut)

# physx3.4
include_directories(3rd_party/physx3.4/Include)
link_directories(3rd_party/physx3.4/Lib/osx64)

LIST(APPEND LIBS
        LowLevelAABB
        LowLevelCloth
        LowLevel
        LowLevelDynamics
        LowLevelParticles
        PhysX3CharacterKinematic
        PhysX3Common
        PhysX3Cooking
        PhysX3
        PhysX3Extensions
        PhysX3Vehicle
        SceneQuery
        SimulationController)

# PxShared
include_directories(3rd_party/PxShared/include)
link_directories(3rd_party/PxShared/lib/osx64)
LIST(APPEND LIBS
        PsFastXml
        PxPvdSDK
        PxFoundation
        PxTask)

# Demos

add_executable(ch2_1_HelloPhysx src/ch2_1_HelloPhysx.cpp)
target_link_libraries(ch2_1_HelloPhysx ${LIBS})

add_executable(ch3_1_Rigidbody src/ch3_1_Rigidbody.cpp)
target_link_libraries(ch3_1_Rigidbody ${LIBS})

add_executable(ch4_1_CollisionDetection src/ch4_1_CollisionDetection.cpp)
target_link_libraries(ch4_1_CollisionDetection ${LIBS})

add_executable(ch5_1_Joints src/ch5_1_Joints.cpp)
target_link_libraries(ch5_1_Joints ${LIBS})

add_executable(ch6_1_SceneQueries src/ch6_1_SceneQueries.cpp)
target_link_libraries(ch6_1_SceneQueries ${LIBS})

add_executable(ch7_1_CharController src/ch7_1_CharController.cpp)
target_link_libraries(ch7_1_CharController ${LIBS})

add_executable(ch8_1_Particles src/ch8_1_Particles.cpp)
target_link_libraries(ch8_1_Particles ${LIBS})

add_executable(ch9_1_Cloth src/ch9_1_Cloth.cpp)
target_link_libraries(ch9_1_Cloth ${LIBS})

