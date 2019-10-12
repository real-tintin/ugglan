# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line, and also
# from the environment for the first two.
SPHINXOPTS    ?=
SPHINXBUILD   = python -m sphinx
SOURCEDIR     = source/
BUILDDIR      = build/

# Build sphinx doc with optional arguments.
.PHONY: build
build:
	$(SPHINXBUILD) "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

# Clean the build dir.
clean:
	rm -r $(BUILDDIR)