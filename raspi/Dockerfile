FROM debian:bullseye AS builder

ARG IMAGE_ROOT
ENV ROOT ${IMAGE_ROOT}
WORKDIR ${ROOT}

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y \
    libz-dev \
    libi2c-dev \
    build-essential \
    libzmq3-dev

FROM builder as clang
RUN apt-get install -y \
    clang-format \
    clang-tidy
