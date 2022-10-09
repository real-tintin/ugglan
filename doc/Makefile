# Makefile for building the Sphinx documentation in
# a docker container.
#

DOC_ROOT = /$(shell pwd)/
IMAGE_ROOT = /doc

UID = $(shell id -u)
GID = $(shell id -u)

SPHINXOPTS ?=
SOURCEDIR   = source/
BUILDDIR    = build/

.DEFAULT_GOAL := build_doc

.PHONY: build_doc build_image build_live

build_image:
	docker build . -t sphinx \
		--build-arg IMAGE_ROOT=${IMAGE_ROOT}

build_doc: build_image
	docker run -v $(DOC_ROOT):$(IMAGE_ROOT) \
		--user $(UID):$(GID) \
		sphinx \
		sphinx-build "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

build_live: build_image
	docker run -it -v $(DOC_ROOT):$(IMAGE_ROOT) \
		--user $(UID):$(GID) \
		-p 8000:8000 \
		sphinx \
		sphinx-autobuild "$(SOURCEDIR)" "$(BUILDDIR)" --host 0.0.0.0 $(SPHINXOPTS) $(O)

clean:
	rm -r $(BUILDDIR)
