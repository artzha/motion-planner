# acceptable build_types: Release/Debug/RelWithDebInfo
build_type=Release

# Extra flags forwarded to the cmake configure step, e.g.
#   make cmake CMAKEFLAGS=-DMOTION_PLANNER_BUILD_APP=OFF
CMAKEFLAGS=

.SILENT:
.PHONY: all cmake run test debug set_debug clean

all: build build/CMakeCache.txt
	$(info Build_type is [${build_type}])
	$(MAKE) --no-print-directory -C build

# Force a reconfigure with the current flags.
cmake: build
	cd build && cmake -DCMAKE_BUILD_TYPE=$(build_type) $(CMAKEFLAGS) ..

build/CMakeCache.txt: build CMakeLists.txt Makefile
	cd build && cmake -DCMAKE_BUILD_TYPE=$(build_type) $(CMAKEFLAGS) ..

build:
	mkdir -p build

# Build then run the demo executable.
run: all
	./build/motion_planner_app

set_debug:
	$(eval build_type=Debug)

debug: | set_debug all

clean:
	rm -rf build
