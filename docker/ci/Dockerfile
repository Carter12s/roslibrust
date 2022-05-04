# We're grabbing an image that already has rosbridge installed / built
FROM joaquingimenez1/noetic-rosbridge:1.1.2 as bridge

# Install Curl
RUN apt update
RUN apt install -y --fix-missing curl
# Install rust
RUN curl https://sh.rustup.rs -sSf | sh -s -- -y

WORKDIR /
COPY entrypoint.sh .
# ENTRYPOINT ["/entrypoint.sh"]
