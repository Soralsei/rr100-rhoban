# RR100 Navigation package
## Dependencies
This project should only depend on docker to be built and function. You can see install instructions by following [this link](https://docs.docker.com/engine/install/).

If you want to have access to hardware acceleration inside the container using an nvidia GPU, follow the install instructions for the nvidia-container-toolkit located [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

## Build
First, clone this repository and its submodules :
```bash
git clone git@github.com:Soralsei/rr100-rhoban.git
cd rr100-rhoban
git submodule update --init --recursive
```

Then, you can simply build the docker image by executing the following command (will take a while):
```bash
docker build . -t <tag-name> --target <simulation|real>
```

## Running-
### With GUI
First, add docker to authorized xhost users :
``` bash
xhost +local:docker
```
Then, run the container :
```bash
docker run --rm -ti \
    --privileged \
    --gpus all \
    --net host \
    --env DISPLAY=$DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    --env XAUTHORITY=$XAUTH \
    --volume "$XAUTH:$XAUTH" \
    -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
    -v $(pwd)/.gazebo:/root/.gazebo/ \
    <image-tag-name>
```

    --device /dev/dri \