# We're grabbing an image that already has rosbridge installed / built
FROM joaquingimenez1/noetic-rosbridge:1.1.2 as bridge
LABEL maintainer="Carter Schultz <carterjschultz@gmail.com>"

# Required for CI
RUN apt update && apt install -y git
# Curl required to install rust, build-essential required to build quote & proc-macro2
RUN apt update && apt install -y --fix-missing curl build-essential
# Install latest stable rust
RUN curl https://sh.rustup.rs -sSf | sh -s -- -y

WORKDIR /
COPY entrypoint.sh .
RUN chmod +x entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
