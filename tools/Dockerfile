FROM python:3.7.9

# Setup env for testing
ENV ROOT /tools
WORKDIR ${ROOT}
COPY . ${ROOT}

RUN pip install --upgrade pip
RUN pip install pytest
RUN pip install .
