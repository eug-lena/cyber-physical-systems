# First stage for building the software:
FROM alpine:latest as builder

ENV LANG C.UTF-8

# Install necessary build tools and dependencies
RUN apk update && \
    apk upgrade && \
    apk add --no-cache \
    ca-certificates \
    cmake \
    g++ \
    make \
    git \
    opencv-dev

# Include the source code and compile
ADD . /opt/sources
WORKDIR /opt/sources
RUN mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/tmp .. && \
    make && make install

# Second stage for packaging the software into a software bundle:
FROM alpine:latest

# Install necessary runtime dependencies
RUN apk update && \
    apk upgrade && \
    apk add --no-cache \
        libstdc++ \
        opencv

# Set up the working directory
WORKDIR /usr/bin

# Copy the compiled binary from the builder stage
COPY --from=builder /tmp/bin/template-opencv .

# Set the entrypoint for the Docker container
ENTRYPOINT ["/usr/bin/template-opencv"]
