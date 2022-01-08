FROM debian:bullseye

# Install dependencies
RUN apt-get update --fix-missing && apt-get upgrade -y
RUN apt-get install -y libz-dev libi2c-dev build-essential cmake

# Copy dependent files (instead of mount to support Windows)
ENV ROOT /raspi
WORKDIR ${ROOT}
COPY . ${ROOT}

# Build iir1 lib and create symbolic link
RUN cd ${ROOT}/lib/iir1 && cmake .
RUN cd ${ROOT}/lib/iir1 && cmake --build . -j8
RUN ln -s ${ROOT}/lib/iir1/libiir_static.a /usr/local/lib/libiir_static.a

# Make entry shell scripts executable
RUN chmod +x ./run_tests.sh
RUN chmod +x ./build_dpkg.sh
