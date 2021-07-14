TARGET_EXEC ?= springsim

BUILD_DIR ?= ./builds/nongui
INC_DIRS ?= ./simulator
SRC_DIRS ?= ./simulator
SRC_LIB_DIRS ?= ./simulator/tclap

EXCLUDE_SRC_FILES := $(SRC_DIRS)/ui/%.cpp
SRCS := $(filter-out $(EXCLUDE_SRC_FILES),$(shell find $(SRC_DIRS) $(SRC_LIB_DIRS) -type f \( -iname \*.cpp -o -iname \*.c \)))
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

INC_DIRS := $(shell find $(SRC_DIRS) -type d)
INC_FLAGS := $(addprefix -I,$(INC_DIRS))

CPPFLAGS ?= $(INC_FLAGS) -MMD -MP -std=gnu++14

$(BUILD_DIR)/$(TARGET_EXEC): $(OBJS)
	$(CXX) $(OBJS) -o $@ $(LDFLAGS)

# c source
$(BUILD_DIR)/%.c.o: %.c
	$(MKDIR_P) $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

# c++ source
$(BUILD_DIR)/%.cpp.o: %.cpp
	$(MKDIR_P) $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

.PHONY: clean

clean:
	$(RM) -r $(BUILD_DIR)

-include $(DEPS)

MKDIR_P ?= mkdir -p