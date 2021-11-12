FROM python:3.7.9

# Setup env for testing
RUN pip install --upgrade pip
RUN pip install pytest

ENV ROOT /tools
WORKDIR ${ROOT}
COPY . ${ROOT}

RUN pip install .
