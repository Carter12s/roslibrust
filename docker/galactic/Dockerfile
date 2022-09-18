FROM ros:galactic-ros-core
LABEL maintainer="Carter Schultz <carterjschultz@gmail.com>"
LABEL org.opencontainers.image.source: "https://github.com/Carter12s/roslibrust"

# Adapted from https://github.com/JoaquinGimenez1/docker-noetic-rosbridge/blob/main/Dockerfile
WORKDIR /
RUN apt update && apt install -y ros-galactic-rosbridge-suite

# Curl required to install rust, build-essential required to build quote & proc-macro2
RUN apt update && apt install -y --fix-missing curl build-essential
# Install rust
RUN curl https://sh.rustup.rs -sSf | sh -s -- -y

# Update Cargo Registry
# See this for why we do it so wierd https://github.com/rust-lang/cargo/issues/3377
RUN /root/.cargo/bin/cargo install lazy_static; exit 0

WORKDIR /
COPY entrypoint.sh .
RUN chmod +x entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]