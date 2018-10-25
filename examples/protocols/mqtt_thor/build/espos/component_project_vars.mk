# Automatically generated build file. Do not edit.
COMPONENT_INCLUDES += $(IDF_PATH)/components/espos/include $(IDF_PATH)/components/espos/include/arch
COMPONENT_LDFLAGS += -L$(BUILD_DIR_BASE)/espos -lespos
COMPONENT_LINKER_DEPS += 
COMPONENT_SUBMODULES += 
COMPONENT_LIBRARIES += espos
component-espos-build: 
