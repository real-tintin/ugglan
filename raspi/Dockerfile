FROM debian:bullseye

# Install dependencies
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y libz-dev libi2c-dev build-essential

# Copy dependent files (instead of mount to support Windows)
ENV ROOT /raspi
WORKDIR ${ROOT}
COPY . ${ROOT}

# Make entry shell scripts executable
RUN chmod +x ./run_tests.sh
RUN chmod +x ./build_dpkg.sh
