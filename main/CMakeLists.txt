idf_component_register(SRCS main.cpp INCLUDE_DIRS "")

target_compile_options(
  ${COMPONENT_LIB}
  PUBLIC "$<$<COMPILE_LANGUAGE:CXX>:-std=gnu++23>" -Wfatal-errors
         -Wno-error=all -Wno-error=extra
         -Wno-error=implicit-function-declaration -Wno-error=format)
