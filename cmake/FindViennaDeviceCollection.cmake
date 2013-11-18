# - Find ViennaDeviceCollection
#
# Defines the following if found:
#   VIENNADEVICECOLLECTION_FOUND          : TRUE if found, FALSE otherwise
#
# Module looks for the path provided by the environment variable
#   VIENNADEVICECOLLECTIONPATH


FIND_PATH(VIENNADEVICECOLLECTION_DIR 
          NAMES definitions.xml
          PATHS $ENV{VIENNADEVICECOLLECTIONPATH}
          )

find_package_handle_standard_args(ViennaDeviceCollection DEFAULT_MSG VIENNADEVICECOLLECTION_DIR)

mark_as_advanced(VIENNADEVICECOLLECTION_DIR)



