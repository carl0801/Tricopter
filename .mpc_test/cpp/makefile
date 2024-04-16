# Compiler
CC := g++
# Compiler flags
CFLAGS := -O3#-Wall
# Include directories 
INCLUDE := -I ./lib
# Source files
SRCS := main.cpp droneModel.cpp
# Object files
OBJS := $(SRCS:.cpp=.o)
# Dependency files
DEPS := $(SRCS:.cpp=.d)
# Executable name
TARGET := main

# Rule to build the executable
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(INCLUDE) $^ -o $@

# Rule to compile source files
%.o: %.cpp
	$(CC) $(CFLAGS) $(INCLUDE) -c $< -o $@

# Include dependency files
-include $(DEPS)

# Rule to generate dependency files
%.d: %.cpp
	@$(CC) -MM $(CFLAGS) $(INCLUDE) $< > $@

# Rule to clean object and dependency files
clean:
	rm -f $(OBJS) $(DEPS)

# Rule to clean all generated files
cleanall: clean
	rm -f $(TARGET)

.PHONY: clean cleanall