FROM debian:bullseye

ARG IMAGE_ROOT
ENV ROOT ${IMAGE_ROOT}
WORKDIR ${ROOT}

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y \
    libz-dev \
    libi2c-dev \
    build-essential \
    libzmq3-dev \
    clang-format \
    clang-tidy
