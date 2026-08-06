if(NOT "${PX4_PLATFORM}" STREQUAL "posix")
	message(FATAL_ERROR "tinympc_fullstate is intentionally limited to PX4 POSIX/SITL")
endif()

get_filename_component(TINYMPC_PX4_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(TINYMPC_CORE "${TINYMPC_PX4_ROOT}/quadtest/tinympc/TinyMPC")

px4_add_module(
	MODULE modules__tinympc_fullstate
	MAIN tinympc_fullstate
	STACK_MAIN 64000
	SRCS
		tinympc_fullstate.cpp
		${TINYMPC_PX4_ROOT}/quadtest/wrapper/tinympc_full_state_interface.cpp
		${TINYMPC_CORE}/src/tinympc/admm.cpp
		${TINYMPC_CORE}/src/tinympc/tiny_api.cpp
		${TINYMPC_CORE}/src/tinympc/rho_benchmark.cpp
	INCLUDES
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
		-Wno-unused-variable
)
