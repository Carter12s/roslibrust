# Noetic CI Dockerfile
Produces a docker image including both noetic rosbridge and needed rust.

# Building / Publishing (currently only carter has access, need to fix)
- docker build -t carter12s/roslibrust-ci-noetic:latest .
- Maybe needed: docker login
- docker push carter12s/roslibrust-ci-noetic:latest