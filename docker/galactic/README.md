# Galactic CI Docker Image
Produces a docker image including both galactic rosbridge and needed rust.

# Building / Publishing (currently only carter has access, need to fix)
- docker build -t carter12s/roslibrust-ci-galactic:latest .
- Maybe needed: docker login
- docker push carter12s/roslibrust-ci-galactic:latest