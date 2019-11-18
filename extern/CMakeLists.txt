###############################################################################
# CMakeLists.txt
#
# Defines build structure for external dependencies
###############################################################################

# Add nlohmann's json library, header-only

add_library(json INTERFACE)

target_include_directories(json
    INTERFACE
        json
)
