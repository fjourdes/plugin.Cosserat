cmake_minimum_required(VERSION 3.1)
project(CosseratPlugin VERSION 1.0)

include(cmake/environment.cmake)
find_package(SofaFramework REQUIRED)

option(COSSERAT_IGNORE_ERRORS "Enable this option to ignore the recommendations and generate anyway." OFF)
if(NOT COSSERAT_IGNORE_ERRORS)    
    if(NOT PLUGIN_SOFAPYTHON)
        message(SEND_ERROR "
            You should not use the plugin Cosserat without enabling PLUGIN_SOFAPYTHON.
            Most examples will not be loadable.
            To fix this error you can either follow the recommendation or enable COSSERAT_IGNORE_ERRORS.")
    endif()    
endif()

find_package(SofaPython QUIET)
set(COSSERAT_PYTHON ${SofaPython_FOUND}) # config.h.in
find_package(STLIB QUIET)
if(STLIB_FOUND)
    message("-- Found dependency : 'STLIB' plugin .")
else()
    message("-- The highly recommanded 'STLIB' plugin is missing. You can compile Cosserat but most of the provided python examples will not work. ")
endif()


set(HEADER_FILES
    src/initCosserat.h

    src/mapping/BaseCosserat.h
    src/mapping/BaseCosserat.inl
    src/mapping/DiscretCosseratMapping.h
    src/mapping/DiscretCosseratMapping.inl
    src/mapping/DiscretDynamicCosseratMapping.h
    src/mapping/DiscretDynamicCosseratMapping.inl
    src/mapping/ProjectionEngine.h
    src/mapping/ProjectionEngine.inl
    src/mapping/DifferenceMultiMapping.h
    src/mapping/DifferenceMultiMapping.inl

    src/forcefield/BeamHookeLawForceField.h
    src/forcefield/BeamHookeLawForceField.inl
    src/forcefield/CosseratInternalActuation.h
    src/forcefield/CosseratInternalActuation.inl

    src/constraint/CosseratSlidingConstraint.h
    src/constraint/CosseratSlidingConstraint.inl
    src/constraint/QPSlidingConstraint.h
    src/constraint/QPSlidingConstraint.inl
    )

set(SOURCE_FILES
    src/initCosserat.cpp

    src/mapping/BaseCosserat.cpp
    src/mapping/DiscretCosseratMapping.cpp

    src/mapping/DiscretDynamicCosseratMapping.cpp
    src/mapping/ProjectionEngine.cpp
    src/mapping/DifferenceMultiMapping.cpp

    src/forcefield/BeamHookeLawForceField.cpp
    src/forcefield/CosseratInternalActuation.cpp

    src/constraint/CosseratSlidingConstraint.cpp 
    src/constraint/QPSlidingConstraint.cpp
    )

set(README_FILE "Cosserat.txt" )

find_package(SoftRobots QUIET)
if(SoftRobots_FOUND)

#     set(HEADER_FILES ${HEADER_FILES}
#        src/constraint/CosseratActuatorConstraint.h
#        src/constraint/CosseratActuatorConstraint.inl
#         )
# 
#     set(SOURCE_FILES ${SOURCE_FILES}
#        src/constraint/CosseratActuatorConstraint.cpp
#         )
    list(APPEND HEADER_FILES
        src/constraint/CosseratActuatorConstraint.h
        src/constraint/CosseratActuatorConstraint.inl
        )
    list(APPEND SOURCE_FILES
        src/constraint/CosseratActuatorConstraint.cpp
        )    

endif()

file(GLOB_RECURSE RESOURCE_FILES  "*.md" "*.psl" "*.py" "*.pyscn" "*.scn" "*.ah")

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${RESOURCE_FILES} )

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include>")

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_COSSERAT")
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")
target_link_libraries(${PROJECT_NAME} SofaCore SofaConstraint SofaSimpleFem SofaBaseMechanics SofaRigid SofaMiscMapping)


if(SofaPython_FOUND)
    target_link_libraries(${PROJECT_NAME} SofaPython)
    # Config files and install rules for pythons scripts
    sofa_install_pythonscripts(PLUGIN_NAME ${PROJECT_NAME} PYTHONSCRIPTS_SOURCE_DIR "python/cosserat")
endif()

if(SoftRobots_FOUND)
    target_link_libraries(${PROJECT_NAME} SoftRobots)
endif() 

## Install rules for the library and headers; CMake package configurations files
sofa_generate_package(
    NAME CosseratPlugin
    VERSION ${PROJECT_VERSION}
    TARGETS ${PROJECT_NAME}
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}    
    RELOCATABLE "plugins"
    )


option(COSSERAT_BUILD_TESTS "Build unit tests" OFF )
if(COSSERAT_BUILD_TESTS)
    add_subdirectory(tests)
endif()

SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)
SET_PROPERTY(TARGET ${PROJECT_NAME} PROPERTY FOLDER "plugins")

include(cmake/packaging.cmake)
