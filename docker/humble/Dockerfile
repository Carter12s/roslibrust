FROM ros:humble-ros-core
LABEL maintainer="Carter Schultz <carterjschultz@gmail.com>"

# Required by github CI for submodule support
RUN apt update && apt install -y git

# Adapted from https://github.com/JoaquinGimenez1/docker-noetic-rosbridge/blob/main/Dockerfile
RUN apt update && apt install -y ros-humble-rosbridge-suite

# Curl required to install rust, build-essential required to build quote & proc-macro2
RUN apt update && apt install -y --fix-missing curl build-essential
# Install latest stable rust
RUN curl https://sh.rustup.rs -sSf | sh -s -- -y

# Update Cargo Registry
# See this for why we do it so wierd https://github.com/rust-lang/cargo/issues/3377
RUN /root/.cargo/bin/cargo install lazy_static; exit 0

WORKDIR /
COPY entrypoint.sh .
RUN chmod +x entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
