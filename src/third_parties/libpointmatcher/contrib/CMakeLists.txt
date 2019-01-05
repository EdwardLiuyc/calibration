# External libraries packed with libpointmatcher

# Wrapped yaml-cpp
if(NOT USE_SYSTEM_YAML_CPP)
add_subdirectory(yaml-cpp-pm)
endif()
