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

ENV CLANG_VERSION 15

RUN apt-get install -y \
    wget \
    lsb-release \
    software-properties-common \
    gnupg

RUN wget https://apt.llvm.org/llvm.sh
RUN chmod +x llvm.sh
RUN ./llvm.sh ${CLANG_VERSION}
RUN apt-get install -y \
    clang-format-${CLANG_VERSION} \
    clang-tidy-${CLANG_VERSION}

RUN ln -sf /usr/bin/clang-format-${CLANG_VERSION} /usr/bin/clang-format
RUN ln -sf /usr/bin/clang-tidy-${CLANG_VERSION} /usr/bin/clang-tidy
