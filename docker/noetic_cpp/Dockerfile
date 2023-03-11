FROM ubuntu:20.04
LABEL maintainer="Carter Schultz <carterjschultz@gmail.com>"

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

RUN apt update
# Install packages required to build Rust crates, C++ test package
RUN apt install -y git g++ curl build-essential cmake
# Install barebones ROS noetic environment
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt update && apt install -y ros-noetic-ros-base
# Install latest stable rust
RUN curl https://sh.rustup.rs -sSf | sh -s -- -y

# Update Cargo Registry
# See this for why we do it so weird https://github.com/rust-lang/cargo/issues/3377
RUN /root/.cargo/bin/cargo install lazy_static; exit 0

WORKDIR /
COPY entrypoint.sh .
RUN chmod +x entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]