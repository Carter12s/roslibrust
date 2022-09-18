# We're grabbing an image that already has rosbridge installed / built
FROM joaquingimenez1/noetic-rosbridge:1.1.2 as bridge
LABEL maintainer="Carter Schultz <carterjschultz@gmail.com>"
LABEL org.opencontainers.image.source: "https://github.com/Carter12s/roslibrust"

# Install Curl
RUN apt update
RUN apt install -y --fix-missing curl
# Install rust
RUN curl https://sh.rustup.rs -sSf | sh -s -- -y

# Update Cargo Registry
# See this for why we do it so wierd https://github.com/rust-lang/cargo/issues/3377
RUN /root/.cargo/bin/cargo install lazy_static; exit 0

WORKDIR /
COPY entrypoint.sh .
RUN chmod +x entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
