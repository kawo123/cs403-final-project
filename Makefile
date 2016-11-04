include $(shell rospack find mk)/cmake.mk

.SILENT:

all: build build/CMakeLists.txt.copy
	$(MAKE) --no-print-directory -C build

build:
	mkdir -p build

build/CMakeLists.txt.copy: CMakeLists.txt msg srv
	cp CMakeLists.txt build/CMakeLists.txt.copy
	cd build && cmake ..

clean:
	rm -rf build bin lib msg_gen srv_gen src/final_main
