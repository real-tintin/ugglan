# Makefile for building python package of tools
# and running unit tests - in docker container.
#

.PHONY: build_pkg_and_test

build_pkg_and_test:
	docker build --platform linux/amd64 -t tools .
	docker run --platform linux/amd64 tools python -m pytest
