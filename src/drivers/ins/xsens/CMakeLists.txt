add_subdirectory(libxsens)

px4_add_module(
	MODULE drivers__ins__xsens
	MAIN xsens
	INCLUDES
		libxsens
	COMPILE_FLAGS
	SRCS
		Xsens.cpp
		Xsens.hpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		libxsens
	)
