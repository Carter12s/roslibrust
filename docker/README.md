## Building and generating the CI docker images
For our CI jobs to work we need an environment with both a rust toolchain, and
a ROS installation sufficient for running rosbridge and rosapi.

These docker files define these images, but we currently don't have CI for updating them.

Instead manually run these commands, and push the images.

`docker build -t carter12s/roslibrust-ci-<image name>:latest .`
`docker push carter12s/roslibrust-ci-<image name>:latest`