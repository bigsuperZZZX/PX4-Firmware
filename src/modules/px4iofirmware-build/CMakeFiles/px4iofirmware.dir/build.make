# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/myros/PX4learn/Firmware/src/modules/px4iofirmware

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/myros/PX4learn/Firmware/src/modules/px4iofirmware-build

# Include any dependencies generated for this target.
include CMakeFiles/px4iofirmware.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/px4iofirmware.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/px4iofirmware.dir/flags.make

CMakeFiles/px4iofirmware.dir/hx_stream.o: CMakeFiles/px4iofirmware.dir/flags.make
CMakeFiles/px4iofirmware.dir/hx_stream.o: /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/hx_stream.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/myros/PX4learn/Firmware/src/modules/px4iofirmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/px4iofirmware.dir/hx_stream.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/px4iofirmware.dir/hx_stream.o   -c /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/hx_stream.c

CMakeFiles/px4iofirmware.dir/hx_stream.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/px4iofirmware.dir/hx_stream.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/hx_stream.c > CMakeFiles/px4iofirmware.dir/hx_stream.i

CMakeFiles/px4iofirmware.dir/hx_stream.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/px4iofirmware.dir/hx_stream.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/hx_stream.c -o CMakeFiles/px4iofirmware.dir/hx_stream.s

CMakeFiles/px4iofirmware.dir/hx_stream.o.requires:

.PHONY : CMakeFiles/px4iofirmware.dir/hx_stream.o.requires

CMakeFiles/px4iofirmware.dir/hx_stream.o.provides: CMakeFiles/px4iofirmware.dir/hx_stream.o.requires
	$(MAKE) -f CMakeFiles/px4iofirmware.dir/build.make CMakeFiles/px4iofirmware.dir/hx_stream.o.provides.build
.PHONY : CMakeFiles/px4iofirmware.dir/hx_stream.o.provides

CMakeFiles/px4iofirmware.dir/hx_stream.o.provides.build: CMakeFiles/px4iofirmware.dir/hx_stream.o


CMakeFiles/px4iofirmware.dir/adc.o: CMakeFiles/px4iofirmware.dir/flags.make
CMakeFiles/px4iofirmware.dir/adc.o: /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/adc.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/myros/PX4learn/Firmware/src/modules/px4iofirmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/px4iofirmware.dir/adc.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/px4iofirmware.dir/adc.o   -c /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/adc.c

CMakeFiles/px4iofirmware.dir/adc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/px4iofirmware.dir/adc.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/adc.c > CMakeFiles/px4iofirmware.dir/adc.i

CMakeFiles/px4iofirmware.dir/adc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/px4iofirmware.dir/adc.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/adc.c -o CMakeFiles/px4iofirmware.dir/adc.s

CMakeFiles/px4iofirmware.dir/adc.o.requires:

.PHONY : CMakeFiles/px4iofirmware.dir/adc.o.requires

CMakeFiles/px4iofirmware.dir/adc.o.provides: CMakeFiles/px4iofirmware.dir/adc.o.requires
	$(MAKE) -f CMakeFiles/px4iofirmware.dir/build.make CMakeFiles/px4iofirmware.dir/adc.o.provides.build
.PHONY : CMakeFiles/px4iofirmware.dir/adc.o.provides

CMakeFiles/px4iofirmware.dir/adc.o.provides.build: CMakeFiles/px4iofirmware.dir/adc.o


CMakeFiles/px4iofirmware.dir/controls.o: CMakeFiles/px4iofirmware.dir/flags.make
CMakeFiles/px4iofirmware.dir/controls.o: /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/controls.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/myros/PX4learn/Firmware/src/modules/px4iofirmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/px4iofirmware.dir/controls.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/px4iofirmware.dir/controls.o   -c /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/controls.c

CMakeFiles/px4iofirmware.dir/controls.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/px4iofirmware.dir/controls.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/controls.c > CMakeFiles/px4iofirmware.dir/controls.i

CMakeFiles/px4iofirmware.dir/controls.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/px4iofirmware.dir/controls.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/controls.c -o CMakeFiles/px4iofirmware.dir/controls.s

CMakeFiles/px4iofirmware.dir/controls.o.requires:

.PHONY : CMakeFiles/px4iofirmware.dir/controls.o.requires

CMakeFiles/px4iofirmware.dir/controls.o.provides: CMakeFiles/px4iofirmware.dir/controls.o.requires
	$(MAKE) -f CMakeFiles/px4iofirmware.dir/build.make CMakeFiles/px4iofirmware.dir/controls.o.provides.build
.PHONY : CMakeFiles/px4iofirmware.dir/controls.o.provides

CMakeFiles/px4iofirmware.dir/controls.o.provides.build: CMakeFiles/px4iofirmware.dir/controls.o


CMakeFiles/px4iofirmware.dir/mixer.o: CMakeFiles/px4iofirmware.dir/flags.make
CMakeFiles/px4iofirmware.dir/mixer.o: /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/mixer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/myros/PX4learn/Firmware/src/modules/px4iofirmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/px4iofirmware.dir/mixer.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/px4iofirmware.dir/mixer.o -c /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/mixer.cpp

CMakeFiles/px4iofirmware.dir/mixer.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/px4iofirmware.dir/mixer.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/mixer.cpp > CMakeFiles/px4iofirmware.dir/mixer.i

CMakeFiles/px4iofirmware.dir/mixer.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/px4iofirmware.dir/mixer.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/mixer.cpp -o CMakeFiles/px4iofirmware.dir/mixer.s

CMakeFiles/px4iofirmware.dir/mixer.o.requires:

.PHONY : CMakeFiles/px4iofirmware.dir/mixer.o.requires

CMakeFiles/px4iofirmware.dir/mixer.o.provides: CMakeFiles/px4iofirmware.dir/mixer.o.requires
	$(MAKE) -f CMakeFiles/px4iofirmware.dir/build.make CMakeFiles/px4iofirmware.dir/mixer.o.provides.build
.PHONY : CMakeFiles/px4iofirmware.dir/mixer.o.provides

CMakeFiles/px4iofirmware.dir/mixer.o.provides.build: CMakeFiles/px4iofirmware.dir/mixer.o


CMakeFiles/px4iofirmware.dir/px4io.o: CMakeFiles/px4iofirmware.dir/flags.make
CMakeFiles/px4iofirmware.dir/px4io.o: /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/px4io.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/myros/PX4learn/Firmware/src/modules/px4iofirmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object CMakeFiles/px4iofirmware.dir/px4io.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/px4iofirmware.dir/px4io.o   -c /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/px4io.c

CMakeFiles/px4iofirmware.dir/px4io.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/px4iofirmware.dir/px4io.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/px4io.c > CMakeFiles/px4iofirmware.dir/px4io.i

CMakeFiles/px4iofirmware.dir/px4io.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/px4iofirmware.dir/px4io.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/px4io.c -o CMakeFiles/px4iofirmware.dir/px4io.s

CMakeFiles/px4iofirmware.dir/px4io.o.requires:

.PHONY : CMakeFiles/px4iofirmware.dir/px4io.o.requires

CMakeFiles/px4iofirmware.dir/px4io.o.provides: CMakeFiles/px4iofirmware.dir/px4io.o.requires
	$(MAKE) -f CMakeFiles/px4iofirmware.dir/build.make CMakeFiles/px4iofirmware.dir/px4io.o.provides.build
.PHONY : CMakeFiles/px4iofirmware.dir/px4io.o.provides

CMakeFiles/px4iofirmware.dir/px4io.o.provides.build: CMakeFiles/px4iofirmware.dir/px4io.o


CMakeFiles/px4iofirmware.dir/registers.o: CMakeFiles/px4iofirmware.dir/flags.make
CMakeFiles/px4iofirmware.dir/registers.o: /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/registers.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/myros/PX4learn/Firmware/src/modules/px4iofirmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object CMakeFiles/px4iofirmware.dir/registers.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/px4iofirmware.dir/registers.o   -c /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/registers.c

CMakeFiles/px4iofirmware.dir/registers.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/px4iofirmware.dir/registers.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/registers.c > CMakeFiles/px4iofirmware.dir/registers.i

CMakeFiles/px4iofirmware.dir/registers.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/px4iofirmware.dir/registers.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/registers.c -o CMakeFiles/px4iofirmware.dir/registers.s

CMakeFiles/px4iofirmware.dir/registers.o.requires:

.PHONY : CMakeFiles/px4iofirmware.dir/registers.o.requires

CMakeFiles/px4iofirmware.dir/registers.o.provides: CMakeFiles/px4iofirmware.dir/registers.o.requires
	$(MAKE) -f CMakeFiles/px4iofirmware.dir/build.make CMakeFiles/px4iofirmware.dir/registers.o.provides.build
.PHONY : CMakeFiles/px4iofirmware.dir/registers.o.provides

CMakeFiles/px4iofirmware.dir/registers.o.provides.build: CMakeFiles/px4iofirmware.dir/registers.o


CMakeFiles/px4iofirmware.dir/safety.o: CMakeFiles/px4iofirmware.dir/flags.make
CMakeFiles/px4iofirmware.dir/safety.o: /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/safety.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/myros/PX4learn/Firmware/src/modules/px4iofirmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object CMakeFiles/px4iofirmware.dir/safety.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/px4iofirmware.dir/safety.o   -c /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/safety.c

CMakeFiles/px4iofirmware.dir/safety.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/px4iofirmware.dir/safety.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/safety.c > CMakeFiles/px4iofirmware.dir/safety.i

CMakeFiles/px4iofirmware.dir/safety.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/px4iofirmware.dir/safety.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/safety.c -o CMakeFiles/px4iofirmware.dir/safety.s

CMakeFiles/px4iofirmware.dir/safety.o.requires:

.PHONY : CMakeFiles/px4iofirmware.dir/safety.o.requires

CMakeFiles/px4iofirmware.dir/safety.o.provides: CMakeFiles/px4iofirmware.dir/safety.o.requires
	$(MAKE) -f CMakeFiles/px4iofirmware.dir/build.make CMakeFiles/px4iofirmware.dir/safety.o.provides.build
.PHONY : CMakeFiles/px4iofirmware.dir/safety.o.provides

CMakeFiles/px4iofirmware.dir/safety.o.provides.build: CMakeFiles/px4iofirmware.dir/safety.o


CMakeFiles/px4iofirmware.dir/serial.o: CMakeFiles/px4iofirmware.dir/flags.make
CMakeFiles/px4iofirmware.dir/serial.o: /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/serial.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/myros/PX4learn/Firmware/src/modules/px4iofirmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building C object CMakeFiles/px4iofirmware.dir/serial.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/px4iofirmware.dir/serial.o   -c /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/serial.c

CMakeFiles/px4iofirmware.dir/serial.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/px4iofirmware.dir/serial.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/serial.c > CMakeFiles/px4iofirmware.dir/serial.i

CMakeFiles/px4iofirmware.dir/serial.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/px4iofirmware.dir/serial.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/myros/PX4learn/Firmware/src/modules/px4iofirmware/serial.c -o CMakeFiles/px4iofirmware.dir/serial.s

CMakeFiles/px4iofirmware.dir/serial.o.requires:

.PHONY : CMakeFiles/px4iofirmware.dir/serial.o.requires

CMakeFiles/px4iofirmware.dir/serial.o.provides: CMakeFiles/px4iofirmware.dir/serial.o.requires
	$(MAKE) -f CMakeFiles/px4iofirmware.dir/build.make CMakeFiles/px4iofirmware.dir/serial.o.provides.build
.PHONY : CMakeFiles/px4iofirmware.dir/serial.o.provides

CMakeFiles/px4iofirmware.dir/serial.o.provides.build: CMakeFiles/px4iofirmware.dir/serial.o


# Object files for target px4iofirmware
px4iofirmware_OBJECTS = \
"CMakeFiles/px4iofirmware.dir/hx_stream.o" \
"CMakeFiles/px4iofirmware.dir/adc.o" \
"CMakeFiles/px4iofirmware.dir/controls.o" \
"CMakeFiles/px4iofirmware.dir/mixer.o" \
"CMakeFiles/px4iofirmware.dir/px4io.o" \
"CMakeFiles/px4iofirmware.dir/registers.o" \
"CMakeFiles/px4iofirmware.dir/safety.o" \
"CMakeFiles/px4iofirmware.dir/serial.o"

# External object files for target px4iofirmware
px4iofirmware_EXTERNAL_OBJECTS =

libpx4iofirmware.a: CMakeFiles/px4iofirmware.dir/hx_stream.o
libpx4iofirmware.a: CMakeFiles/px4iofirmware.dir/adc.o
libpx4iofirmware.a: CMakeFiles/px4iofirmware.dir/controls.o
libpx4iofirmware.a: CMakeFiles/px4iofirmware.dir/mixer.o
libpx4iofirmware.a: CMakeFiles/px4iofirmware.dir/px4io.o
libpx4iofirmware.a: CMakeFiles/px4iofirmware.dir/registers.o
libpx4iofirmware.a: CMakeFiles/px4iofirmware.dir/safety.o
libpx4iofirmware.a: CMakeFiles/px4iofirmware.dir/serial.o
libpx4iofirmware.a: CMakeFiles/px4iofirmware.dir/build.make
libpx4iofirmware.a: CMakeFiles/px4iofirmware.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/myros/PX4learn/Firmware/src/modules/px4iofirmware-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Linking CXX static library libpx4iofirmware.a"
	$(CMAKE_COMMAND) -P CMakeFiles/px4iofirmware.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/px4iofirmware.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/px4iofirmware.dir/build: libpx4iofirmware.a

.PHONY : CMakeFiles/px4iofirmware.dir/build

CMakeFiles/px4iofirmware.dir/requires: CMakeFiles/px4iofirmware.dir/hx_stream.o.requires
CMakeFiles/px4iofirmware.dir/requires: CMakeFiles/px4iofirmware.dir/adc.o.requires
CMakeFiles/px4iofirmware.dir/requires: CMakeFiles/px4iofirmware.dir/controls.o.requires
CMakeFiles/px4iofirmware.dir/requires: CMakeFiles/px4iofirmware.dir/mixer.o.requires
CMakeFiles/px4iofirmware.dir/requires: CMakeFiles/px4iofirmware.dir/px4io.o.requires
CMakeFiles/px4iofirmware.dir/requires: CMakeFiles/px4iofirmware.dir/registers.o.requires
CMakeFiles/px4iofirmware.dir/requires: CMakeFiles/px4iofirmware.dir/safety.o.requires
CMakeFiles/px4iofirmware.dir/requires: CMakeFiles/px4iofirmware.dir/serial.o.requires

.PHONY : CMakeFiles/px4iofirmware.dir/requires

CMakeFiles/px4iofirmware.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/px4iofirmware.dir/cmake_clean.cmake
.PHONY : CMakeFiles/px4iofirmware.dir/clean

CMakeFiles/px4iofirmware.dir/depend:
	cd /home/myros/PX4learn/Firmware/src/modules/px4iofirmware-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/myros/PX4learn/Firmware/src/modules/px4iofirmware /home/myros/PX4learn/Firmware/src/modules/px4iofirmware /home/myros/PX4learn/Firmware/src/modules/px4iofirmware-build /home/myros/PX4learn/Firmware/src/modules/px4iofirmware-build /home/myros/PX4learn/Firmware/src/modules/px4iofirmware-build/CMakeFiles/px4iofirmware.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/px4iofirmware.dir/depend

