get_filename_component(TINYMPC_PX4_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(TINYMPC_CORE "${TINYMPC_PX4_ROOT}/quadtest/tinympc/TinyMPC")
# Embed a content identifier as well as the base commit, so uncommitted builds
# can be matched to the packaged manifest and the recorded ULog.
file(GLOB_RECURSE TINYMPC_ID_FILES CONFIGURE_DEPENDS
	"${TINYMPC_PX4_ROOT}/px4_external/*.cpp"
	"${TINYMPC_PX4_ROOT}/px4_external/*.msg"
	"${TINYMPC_PX4_ROOT}/px4_external/*.h.in"
	"${TINYMPC_PX4_ROOT}/px4_external/*CMakeLists.txt"
	"${TINYMPC_PX4_ROOT}/quadtest/wrapper/*.h"
	"${TINYMPC_PX4_ROOT}/quadtest/wrapper/*.hpp"
	"${TINYMPC_PX4_ROOT}/quadtest/wrapper/tinympc_interface.cpp"
	"${TINYMPC_CORE}/src/tinympc/*.cpp"
	"${TINYMPC_CORE}/src/tinympc/*.hpp"
)
set(TINYMPC_ID_INPUT "")
foreach(TINYMPC_ID_FILE IN LISTS TINYMPC_ID_FILES)
	set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${TINYMPC_ID_FILE}")
	file(SHA256 "${TINYMPC_ID_FILE}" TINYMPC_FILE_HASH)
	file(RELATIVE_PATH TINYMPC_ID_PATH "${TINYMPC_PX4_ROOT}" "${TINYMPC_ID_FILE}")
	string(APPEND TINYMPC_ID_INPUT "${TINYMPC_ID_PATH}:${TINYMPC_FILE_HASH}\n")
endforeach()
string(SHA256 TINYMPC_SOURCE_HASH "${TINYMPC_ID_INPUT}")
string(SUBSTRING "${TINYMPC_SOURCE_HASH}" 0 16 TINYMPC_SOURCE_ID)
execute_process(COMMAND git -C "${TINYMPC_PX4_ROOT}" rev-parse --short=12 HEAD
	OUTPUT_VARIABLE TINYMPC_SOURCE_COMMIT OUTPUT_STRIP_TRAILING_WHITESPACE)
configure_file(tinympc_build_info.h.in tinympc_build_info.h @ONLY)
set(TINYMPC_CHICANE_SOURCES
	tinympc_chicane.cpp
	${TINYMPC_PX4_ROOT}/quadtest/wrapper/tinympc_interface.cpp
	${TINYMPC_CORE}/src/tinympc/admm.cpp
	${TINYMPC_CORE}/src/tinympc/tiny_api.cpp
)

if(NOT "${PX4_PLATFORM}" STREQUAL "nuttx")
	list(APPEND TINYMPC_CHICANE_SOURCES
		${TINYMPC_CORE}/src/tinympc/rho_benchmark.cpp
	)
endif()

px4_add_module(
	MODULE modules__tinympc_chicane
	MAIN tinympc_chicane
	STACK_MAIN 64000
	EXTERNAL
	SRCS
		${TINYMPC_CHICANE_SOURCES}
	INCLUDES
		${CMAKE_CURRENT_BINARY_DIR}
		${TINYMPC_PX4_ROOT}/quadtest/wrapper
		${TINYMPC_CORE}/src
		${TINYMPC_CORE}/src/tinympc
		${TINYMPC_CORE}/include/Eigen
	COMPILE_FLAGS
		-fpermissive
		-Wno-address-of-packed-member
		-Wno-cast-align
		-Wno-double-promotion
		-Wno-float-equal
		-Wno-unused-function
		-Wno-unused-variable
		-DMPC_Init=tinympc_chicane_MPC_Init
		-DMPC_Reset=tinympc_chicane_MPC_Reset
		-DMPC_Step=tinympc_chicane_MPC_Step
		-DMPC_Step_Plan=tinympc_chicane_MPC_Step_Plan
		-DMPC_Step_Scenario=tinympc_chicane_MPC_Step_Scenario
)

# PX4/NuttX intentionally builds without the toolchain's C++ include paths.
# Eigen is header-only, but it still needs standard template headers such as
# <algorithm>, <complex>, and <type_traits>. Add headers only; the firmware
# continues to link against NuttX/PX4 runtime libraries, not libstdc++.
if("${PX4_PLATFORM}" STREQUAL "nuttx")
	execute_process(
		COMMAND ${CMAKE_CXX_COMPILER} -dumpfullversion
		OUTPUT_VARIABLE TINYMPC_GCC_VERSION
		OUTPUT_STRIP_TRAILING_WHITESPACE
	)
	get_filename_component(TINYMPC_COMPILER_BIN "${CMAKE_CXX_COMPILER}" DIRECTORY)
	set(TINYMPC_STDCPP_CANDIDATES
		"${TINYMPC_COMPILER_BIN}/../arm-none-eabi/include/c++/${TINYMPC_GCC_VERSION}"
		"${TINYMPC_COMPILER_BIN}/../include/c++/${TINYMPC_GCC_VERSION}"
		"${TINYMPC_COMPILER_BIN}/../include/newlib/c++/${TINYMPC_GCC_VERSION}"
		"/usr/include/newlib/c++/${TINYMPC_GCC_VERSION}"
	)
	foreach(TINYMPC_STDCPP_CANDIDATE IN LISTS TINYMPC_STDCPP_CANDIDATES)
		if(EXISTS "${TINYMPC_STDCPP_CANDIDATE}/complex")
			get_filename_component(TINYMPC_STDCPP "${TINYMPC_STDCPP_CANDIDATE}" ABSOLUTE)
			break()
		endif()
	endforeach()

	if(NOT TINYMPC_STDCPP)
		message(FATAL_ERROR "TinyMPC could not locate the ARM C++ headers")
	endif()

	# Use explicit compiler options: CMake otherwise removes these directories
	# as "implicit" even though PX4 later adds -nostdinc++.
	target_compile_options(modules__tinympc_chicane PRIVATE
		"-I${TINYMPC_PX4_ROOT}/px4_external/include"
		"-I${TINYMPC_STDCPP}"
		"-I${TINYMPC_STDCPP}/arm-none-eabi"
		"-I${TINYMPC_STDCPP}/backward"
	)
	target_compile_definitions(modules__tinympc_chicane PRIVATE
		EIGEN_NO_IO
		EIGEN_DONT_VECTORIZE
		TINY_DISABLE_ADAPTIVE_RHO
		TINY_MPC_HARDWARE_HOLD_ONLY
		TINY_MPC_SINGLE_PRECISION
		TINY_MPC_USE_STATIC_CACHE
	)
endif()
