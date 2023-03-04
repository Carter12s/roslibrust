# Noetic C++ CI Dockerfile
Produces a docker image including barebones noetic environment and C++ compiler.

# Building / Publishing (currently only carter has access, need to fix)
- docker build -t carter12s/roslibrust-ci-noetic-cpp:latest .
- Maybe needed: docker login
- docker push carter12s/roslibrust-ci-noetic-cpp:latest

For debug:
docker run -it carter12s/roslibrust-ci-noetic-cpp /bin/bash