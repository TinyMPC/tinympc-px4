if(NOT "${PX4_PLATFORM}" STREQUAL "posix")
	message(FATAL_ERROR "tinympc_chicane is intentionally limited to PX4 POSIX/SITL")
endif()

get_filename_component(TINYMPC_PX4_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(TINYMPC_CORE "${TINYMPC_PX4_ROOT}/quadtest/tinympc/TinyMPC")

px4_add_module(
	MODULE modules__tinympc_chicane
	MAIN tinympc_chicane
	STACK_MAIN 64000
	SRCS
		tinympc_chicane.cpp
		${TINYMPC_PX4_ROOT}/quadtest/wrapper/tinympc_interface.cpp
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
		-DMPC_Init=tinympc_chicane_MPC_Init
		-DMPC_Reset=tinympc_chicane_MPC_Reset
		-DMPC_Step=tinympc_chicane_MPC_Step
		-DMPC_Step_Plan=tinympc_chicane_MPC_Step_Plan
		-DMPC_Step_Scenario=tinympc_chicane_MPC_Step_Scenario
)
