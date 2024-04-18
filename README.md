# RR100 Navigation package
## Dependencies
This project should only depend on docker to be built and function. You can see install instructions by following [this link](https://docs.docker.com/engine/install/).

If you want to have access to hardware acceleration inside the container using an nvidia GPU, follow the install instructions for the nvidia-container-toolkit located [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

## Building and running
First, clone this repository and its submodules :
```console
user@machine:~$ git clone git@github.com:Soralsei/rr100-rhoban.git
user@machine:~$ cd rr100-rhoban
user@machine:~/rr100-rhoban$ git submodule update --init --recursive
```

Then, for convenience, a bash script (`rr100-rhoban/build_and_run.sh`) to build and run the project is provided at the root of the repository.

Usage example  :
```console
user@machine:~/rr100-rhoban$ ./build_and_run.sh --image-tag rr100-sim --target simulation -g
```

### Script parameters
```
Usage : build_and_run.sh [OPTIONS]
Options :
    -i, --image-tag <tag-name>              Used to specify the docker image to run/build REQUIRED
    -c, --container-name <container-name>   Used to specify the docker container name when running the image, OPTIONAL, default: 'ros'
    -r, --rebuild                           Rebuild the image, OPTIONAL, default: false
    -t, --target [real, simulation]         The build target, OPTIONAL, default: 'real'
    -g, --gpu                               Add GPU support, OPTIONAL, default: false
    -h, --help                              Show this message
```