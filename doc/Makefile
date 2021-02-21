# Makefile for building the Sphinx documentation in
# a docker container.
#

DOC_ROOT = /$(shell pwd)/
DOC_ROOT_MNT = /doc

SPHINXOPTS  ?=
SPHINXBUILD = python -m sphinx
SOURCEDIR   = source/
BUILDDIR    = build/

.PHONY: build
build:
	docker build . -t sphinx
	docker run -v $(DOC_ROOT):$(DOC_ROOT_MNT) \
			sphinx \
			$(SPHINXBUILD) "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

clean:
	rm -r $(BUILDDIR)
