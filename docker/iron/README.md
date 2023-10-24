# Iron CI Docker Image
Produces a docker image including both iron rosbridge and needed rust.

# Building / Publishing (currently only carter has access, need to fix)
- docker build -t carter12s/roslibrust-ci-iron:latest .
- Maybe needed: docker login
- docker push carter12s/roslibrust-ci-iron:latest

For debug:
docker run -it carter12s/roslibrust-ci-iron /bin/bash
