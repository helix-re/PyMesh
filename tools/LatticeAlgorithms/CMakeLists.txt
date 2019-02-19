# Source files
FILE(GLOB SRC_FILES *.cpp)
FILE(GLOB INC_FILES *.h)

ADD_LIBRARY(lib_LatticeAlgorithms SHARED ${SRC_FILES} ${INC_FILES})
TARGET_LINK_LIBRARIES(lib_LatticeAlgorithms
    PUBLIC
        PyMesh::Tools
    PRIVATE
        lib_MeshUtils
        PyMesh::Tools::CGAL
)
SET_TARGET_PROPERTIES(lib_LatticeAlgorithms PROPERTIES OUTPUT_NAME
    "PyMesh-LatticeAlgorithms")

ADD_LIBRARY(PyMesh::Tools::LatticeAlgorithms ALIAS lib_LatticeAlgorithms)