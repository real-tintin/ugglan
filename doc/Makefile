# Makefile for building the Sphinx documentation in
# a docker container.
#

DOC_ROOT = /$(shell pwd)/
DOC_ROOT_MNT = /doc

SPHINXOPTS ?=
SOURCEDIR   = source/
BUILDDIR    = build/

.DEFAULT_GOAL := build_doc

.PHONY: build_doc build_image build_live

build_image:
	docker build . -t sphinx

build_doc: build_image
	docker run -v $(DOC_ROOT):$(DOC_ROOT_MNT) \
			sphinx \
			sphinx-build "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

build_live: build_image
	docker run -it -v $(DOC_ROOT):$(DOC_ROOT_MNT) \
			-p 8000:8000 \
			sphinx \
			sphinx-autobuild "$(SOURCEDIR)" "$(BUILDDIR)" --host 0.0.0.0 $(SPHINXOPTS) $(O)

clean:
	rm -r $(BUILDDIR)
