FROM python:3.9.5

RUN apt-get update
RUN apt-get install ffmpeg libsm6 libxext6 -y

RUN pip install --upgrade pip

ENV ROOT /tools
WORKDIR ${ROOT}
COPY . ${ROOT}

RUN pip install .[test]
