FROM python:3.9.5

ARG IMAGE_ROOT
ENV ROOT ${IMAGE_ROOT}
WORKDIR ${ROOT}

RUN pip install --upgrade pip
RUN pip install sphinx sphinx-autobuild sphinxcontrib-mermaid
