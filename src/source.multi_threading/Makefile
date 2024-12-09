## header-start
################################################################################
##
## \file      Makefile
## \brief     Robot Inverse Kinematic with Cyclic Coordinate Descent
##
################################################################################
## header-end
##

.SUFFIXES:
MAKEFLAGS += --silent
SHELL = /bin/bash

target := $(shell fgrep 'set(PRJNAME' CMakeLists.txt | sed 's/)//' | awk '{print $$2}')
testexe := "dotest"



default_target: debug


## to keep colors
## unbuffer make |& less -R  

help:
	@echo "make test          # compile the release version"
	@echo "make release       # compile the release version"
	@echo "make debug         # compile the debug version"
	@echo "make sanitize      # compile the sanitize version (leak)"
	@echo "make all           # make all versions"
	@echo "make clean         # full cleanup back to initial install state"
	@echo "make initrelease   # cleanup and init release"
	@echo "make initdebug     # cleanup and init debug"
	@echo "make initsanitize  # cleanup and init sanitize"
.PHONY: help



release:| initrelease
	@pushd build/release > /dev/null;\
	cmake --build . -- all && popd > /dev/null && cp build/release/src/$(target) .
	@if [ -e $(target) ]; then \
		echo "[100%] Target ./$(target) is ready"; \
	fi
.PHONY: release

debug:| initdebug
	@pushd build/debug > /dev/null;\
	cmake --build . -- all && popd > /dev/null && cp build/debug/src/$(target) $(target).debug
	@if [ -e $(target).debug ]; then \
		echo "[100%] Target ./$(target).debug is ready"; \
	fi
.PHONY: debug

# test: ./build/dotest/test/unit_test
#
dotest:| initdotest
	@pushd build/dotest > /dev/null;\
	cmake --build . -- all && popd > /dev/null && cp build/dotest/test/unit_test .
	@if [ -e unit_test ]; then \
		echo "[100%] Target ./unit_test.dotest is ready"; \
	fi
.PHONY: dotest

sanitize:| initsanitize
	@pushd build/sanitize > /dev/null;\
	cmake --build . -- all && popd > /dev/null && cp build/sanitize/src/$(target) $(target).sanitize
	@if [ -e $(target).sanitize ]; then \
		echo "[100%] Target ./$(target).sanitize is ready"; \
	fi
.PHONY: sanitize


all: initrelease release initdebug debug initsanitize sanitize
.PHONY: all

clean:
	rm -fr build
	@rm -f $(target) $(target).debug $(target).sanitize
.PHONY: clean


initdebug:
	if [ ! -d build/debug ]; then \
		mkdir -p build/debug; \
		cd build/debug; \
		cmake -DCMAKE_BUILD_TYPE=Debug ../..; \
  fi
.PHONY: initdebug

initrelease:
	if [ ! -d build/release ]; then \
		mkdir -p build/release; \
		cd build/release; \
		cmake -DCMAKE_BUILD_TYPE=Release ../..; \
  fi
.PHONY: initrelease

initsanitize:
	if [ ! -d build/sanitize ]; then \
		mkdir -p build/sanitize; \
		cd build/sanitize; \
		cmake -DCMAKE_BUILD_TYPE=Sanitize ../..; \
  fi
.PHONY: initsanitize

initdotest:
	if [ ! -d build/dotest ]; then \
		mkdir -p build/dotest; \
  fi; \
	cd build/dotest; \
	cmake -DENABLE_TEST=1 -DCMAKE_BUILD_TYPE=Debug ../.. 

.PHONY: initdotest


fullinit: cleanall fullinitifneeded
.PHONY: fullinit


# help on Makefile
#
# Silence nested Make
#$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
#$(VERBOSE).SILENT:
#targets : prerequisites
#	        recipe
