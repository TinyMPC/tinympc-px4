# The acceleration-level module is the only hardware-port candidate. Keep the
# deeper torque/thrust controller out of every NuttX image until its model and
# motor scaling have been validated on a physical airframe.
set(tinympc_external_modules
	tinympc_chicane
)

if("${PX4_PLATFORM}" STREQUAL "posix")
	list(APPEND tinympc_external_modules tinympc_fullstate)
endif()

set(config_module_list_external ${tinympc_external_modules} PARENT_SCOPE)
