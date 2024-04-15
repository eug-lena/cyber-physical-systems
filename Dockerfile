##################################################
# Section 1: Build the application
FROM ubuntu:22.04 as builder
MAINTAINER Christian Berger christian.berger@gu.se

# Prevent interactive prompts during package installation (Chat-gpt suggested this)
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y && \
    apt-get upgrade -y && \
    apt-get dist-upgrade -y

RUN apt-get install -y --no-install-recommends \
        cmake \
        build-essential

RUN apt install gcovr -y

# Push the src directory to the Docker container
ADD ./src/ /opt/sources
WORKDIR /opt/sources

RUN cd /opt/sources && \
    mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release .. && \
    make && make test && cp helloworld /tmp && \
    gcovr --xml-pretty --exclude-unreachable-branches --exclude='.*\.hpp' --exclude='.*usr/include/.*' --exclude='.*Test[A-Z|a-z]*\.cpp' --print-summary -o coverage.xml --root .. && \
    cp coverage.xml /tmp

##################################################
# Section 2: Copy the application to a new image.
FROM scratch as temp
COPY --from=builder /tmp/coverage.xml .

##################################################
# Section 3: Bundle the application.
FROM ubuntu:22.04
MAINTAINER Christian Berger christian.berger@gu.se

RUN apt-get update -y && \
    apt-get upgrade -y && \
    apt-get dist-upgrade -y

WORKDIR /opt
COPY --from=builder /tmp/helloworld .
COPY --from=builder /tmp/coverage.xml .
ENTRYPOINT ["/opt/helloworld"]
