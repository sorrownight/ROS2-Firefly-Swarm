# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/remigor/hen_ichaer/src/firefly/plugins

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/remigor/hen_ichaer/src/firefly/plugins

# Include any dependencies generated for this target.
include CMakeFiles/FireflyLED.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/FireflyLED.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/FireflyLED.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/FireflyLED.dir/flags.make

CMakeFiles/FireflyLED.dir/FireflyLED.cc.o: CMakeFiles/FireflyLED.dir/flags.make
CMakeFiles/FireflyLED.dir/FireflyLED.cc.o: FireflyLED.cc
CMakeFiles/FireflyLED.dir/FireflyLED.cc.o: CMakeFiles/FireflyLED.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/remigor/hen_ichaer/src/firefly/plugins/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/FireflyLED.dir/FireflyLED.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/FireflyLED.dir/FireflyLED.cc.o -MF CMakeFiles/FireflyLED.dir/FireflyLED.cc.o.d -o CMakeFiles/FireflyLED.dir/FireflyLED.cc.o -c /home/remigor/hen_ichaer/src/firefly/plugins/FireflyLED.cc

CMakeFiles/FireflyLED.dir/FireflyLED.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FireflyLED.dir/FireflyLED.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/remigor/hen_ichaer/src/firefly/plugins/FireflyLED.cc > CMakeFiles/FireflyLED.dir/FireflyLED.cc.i

CMakeFiles/FireflyLED.dir/FireflyLED.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FireflyLED.dir/FireflyLED.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/remigor/hen_ichaer/src/firefly/plugins/FireflyLED.cc -o CMakeFiles/FireflyLED.dir/FireflyLED.cc.s

# Object files for target FireflyLED
FireflyLED_OBJECTS = \
"CMakeFiles/FireflyLED.dir/FireflyLED.cc.o"

# External object files for target FireflyLED
FireflyLED_EXTERNAL_OBJECTS =

libFireflyLED.so: CMakeFiles/FireflyLED.dir/FireflyLED.cc.o
libFireflyLED.so: CMakeFiles/FireflyLED.dir/build.make
libFireflyLED.so: /usr/lib/x86_64-linux-gnu/libgz-sim7.so.7.4.0
libFireflyLED.so: /usr/lib/x86_64-linux-gnu/libgz-common5-profiler.so.5.4.0
libFireflyLED.so: /usr/lib/x86_64-linux-gnu/libgz-fuel_tools8.so.8.0.2
libFireflyLED.so: /usr/lib/x86_64-linux-gnu/libgz-gui7.so.7.1.0
libFireflyLED.so: /usr/lib/x86_64-linux-gnu/libgz-common5-events.so.5.4.0
libFireflyLED.so: /usr/lib/x86_64-linux-gnu/libgz-common5.so.5.4.0
libFireflyLED.so: /usr/lib/x86_64-linux-gnu/libgz-plugin2-loader.so.2.0.1
libFireflyLED.so: /usr/lib/x86_64-linux-gnu/libgz-plugin2.so.2.0.1
libFireflyLED.so: /usr/lib/x86_64-linux-gnu/libQt5QuickControls2.so.5.15.3
libFireflyLED.so: /usr/lib/x86_64-linux-gnu/libQt5Quick.so.5.15.3
libFireflyLED.so: /usr/lib/x86_64-linux-gnu/libQt5QmlModels.so.5.15.3
libFireflyLED.so: /usr/lib/x86_64-linux-gnu/libQt5Qml.so.5.15.3
libFireflyLED.so: /usr/lib/x86_64-linux-gnu/libQt5Network.so.5.15.3
libFireflyLED.so: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.15.3
libFireflyLED.so: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.15.3
libFireflyLED.so: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.15.3
libFireflyLED.so: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
libFireflyLED.so: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
libFireflyLED.so: /usr/lib/x86_64-linux-gnu/libgz-transport12-parameters.so.12.2.0
libFireflyLED.so: /usr/lib/x86_64-linux-gnu/libgz-transport12.so.12.2.0
libFireflyLED.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libFireflyLED.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libFireflyLED.so: /usr/lib/x86_64-linux-gnu/libgz-msgs9.so.9.4.0
libFireflyLED.so: /usr/lib/x86_64-linux-gnu/libsdformat13.so.13.4.1
libFireflyLED.so: /usr/lib/x86_64-linux-gnu/libgz-math7.so.7.1.0
libFireflyLED.so: /usr/lib/x86_64-linux-gnu/libgz-utils2.so.2.0.0
libFireflyLED.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libFireflyLED.so: CMakeFiles/FireflyLED.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/remigor/hen_ichaer/src/firefly/plugins/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libFireflyLED.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/FireflyLED.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/FireflyLED.dir/build: libFireflyLED.so
.PHONY : CMakeFiles/FireflyLED.dir/build

CMakeFiles/FireflyLED.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/FireflyLED.dir/cmake_clean.cmake
.PHONY : CMakeFiles/FireflyLED.dir/clean

CMakeFiles/FireflyLED.dir/depend:
	cd /home/remigor/hen_ichaer/src/firefly/plugins && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/remigor/hen_ichaer/src/firefly/plugins /home/remigor/hen_ichaer/src/firefly/plugins /home/remigor/hen_ichaer/src/firefly/plugins /home/remigor/hen_ichaer/src/firefly/plugins /home/remigor/hen_ichaer/src/firefly/plugins/CMakeFiles/FireflyLED.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/FireflyLED.dir/depend
