#Generate the local Makefile
if(CREATE_LOCAL_MAKEFILES)
  file(RELATIVE_PATH CISPBA_RELATIVE_PATH ${CISPBA_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
  configure_file(${CISPBA_SOURCE_DIR}/Scripts/Makefile.in ${CMAKE_CURRENT_SOURCE_DIR}/Makefile @ONLY)
endif()


#file(GLOB FEM_SRC "src/*.h" "src/*.cpp")

#add_cispba_executable(example ${FEM_SRC})


add_cispba_executable(example src/main.cpp
        src/FEMSolver.cpp
        src/FEMSolver.h
        src/integrator/ForwardEuler.h
        src/integrator/ForwardEuler.cpp
        src/integrator/BaseIntegrator.h
        src/integrator/BaseIntegrator.cpp
        src/components/Spring.h
        src/components/Spring.cpp
        src/mesh/Mesh.h
        src/mesh/Particles.h
        src/mesh/TetraMesh.h
        src/mesh/Tetrahedron.h
        src/utility/FileHelper.cpp
        src/utility/FileHelper.h
        src/globalincludes.h
        src/objects/cube.1.edge
        src/objects/cube.1.ele
        src/objects/cube.1.face
        src/objects/cube.1.node
        src/objects/cube.poly
        src/scene/squareplane.h
        src/scene/shape.h)

include_directories(Eigen)

include_directories(src)

target_link_libraries(example partio)
