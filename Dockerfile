##################################################
# Section 1: Build the application
FROM alpine:latest as builder

# Prevent interactive prompts during package installation (Chat-gpt suggested this)
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
    opencv-dev \
    build-base \
    gcovr

# Include the source code and compile
ADD . /opt/sources
WORKDIR /opt/sources
RUN mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/tmp .. && \
    make && make install && \
    make test && \
    gcovr --xml-pretty --exclude-unreachable-branches --exclude='.*\.hpp' --exclude='.*usr/include/.*' --exclude='.*Test[A-Z|a-z]*\.cpp' --print-summary -o coverage.xml --root .. && \
    gcovr --exclude='.*\.hpp' --exclude='.*usr/include/.*' --exclude='.*Test[A-Z|a-z]*\.cpp' --print-summary -r .. && \
    cp coverage.xml /tmp

##################################################
# Section 2: Copy the application to a new image.
FROM scratch as temp
COPY --from=builder /tmp/coverage.xml .

##################################################
# Section 3: Bundle the application.
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
