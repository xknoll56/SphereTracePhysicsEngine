cmake_minimum_required(VERSION 3.10)
project(SphereTrace)

# Set C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Source files
set(SOURCES
    SphereTrace.c
    SphereTraceAI.c
    SphereTraceAllocator.c
    SphereTraceCollider.c
    SphereTraceColliderBox.c
    SphereTraceColliderPlane.c
    SphereTraceColliderSphere.c
    SphereTraceColliderTerrain.c
    SphereTraceColliderTriangle.c
    SphereTraceLists.c
    SphereTraceMaterial.c
    SphereTraceMath.c
    SphereTraceRigidBody.c
    SphereTraceSpacialPartition.c
    SphereTraceTag.c
)

# Create static library
add_library(SphereTrace STATIC ${SOURCES})

# Set output name to match requested name
set_target_properties(SphereTrace PROPERTIES OUTPUT_NAME "SphereTrace")

# Compiler options
if(MSVC)
    target_compile_options(SphereTrace PRIVATE /W3 /O2 /EHsc)
    # Use static runtime by default, but can be changed
    set_property(TARGET SphereTrace PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
else()
    target_compile_options(SphereTrace PRIVATE -Wall -O2)
endif()

# Include directories (if needed for headers)
target_include_directories(SphereTrace PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

