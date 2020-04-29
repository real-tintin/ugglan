# Makefile for compiling and running C++ catch
# tests in docker container.
#

DRONE_SOURCE_PATH = /$(shell pwd)/..

.PHONY: run_tests_in_docker

run_tests_in_docker:
	docker build . -t catch2
	docker run -v $(DRONE_SOURCE_PATH):/drone catch2 /drone/tests/run_catch_tests.sh
