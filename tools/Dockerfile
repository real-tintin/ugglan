FROM python:3.10.9

RUN apt-get update
RUN apt-get install ffmpeg libsm6 libxext6 libegl1-mesa libegl1 -y

RUN pip install --upgrade pip

ENV ROOT /tools
WORKDIR ${ROOT}
COPY . ${ROOT}

RUN pip install .[test]
