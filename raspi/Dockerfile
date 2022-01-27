FROM debian:bullseye

# Install dependencies
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y libz-dev libi2c-dev build-essential

# Copy dependent files (instead of mount to support Windows)
ENV ROOT /raspi
WORKDIR ${ROOT}
COPY . ${ROOT}
