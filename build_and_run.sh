#!/bin/bash
usage() {
    echo "Usage : build_and_run.sh [OPTIONS]
Options :
    -i, --image-tag <tag-name>              Used to specify the docker image to run/build REQUIRED
    -c, --container-name <container-name>   Used to specify the docker container name when running the image, OPTIONAL, default: 'ros'
    -r, --rebuild                           Rebuild the image, OPTIONAL, default: false
    -t, --target [real, simulation]         The build target, OPTIONAL, default: 'real'
    -a --ip-address                         Your desired ROS_IP address, only used if target=='real'
    -g, --gpu                               Add GPU support, OPTIONAL, default: false
    -h, --help                              Show this message"
    exit 2
}

if [[ $# = 0 ]]; then
    usage
fi

args=()
# Convert long-hand arguments (eg. --image-tag -> -i) and keep the rest as-is
for arg in "$@"; do
    case "$arg" in
    --help) args+=(-h) ;;
    --image-tag) args+=(-i) ;;
    --container-name) args+=(-c) ;;
    --rebuild) args+=(-r) ;;
    --target) args+=(-t) ;;
    --gpu) args+=(-g) ;;
    --ip-address) args+=(-a) ;;
    *) args+=("$arg") ;;
    esac
done
# Replace the input arguments with the converted ones
set -- "${args[@]}"

# Default variables initialization
builder="builder"
target="real"
tag=""
rebuild=false
gpu=false
container="ros"
ros_ip=""

# Input argument parsing
while getopts t:b:hi:rgc:a: option; do
    : "$option" "$OPTARG"
    case $option in
    i)
        tag="$OPTARG"
        # If this option with required a value is missing said value, echo an error and exit
        if [[ ${OPTARG:0:1} == '-' ]]; then
            echo "Invalid value '$OPTARG' given to -$option" >&2
            exit 3
        fi
        ;;
    t)
        target="$OPTARG"
        # If this option with required a value is missing said value, echo an error and exit
        if [[ ${OPTARG:0:1} == '-' ]]; then
            echo "Invalid value '$OPTARG' given to -$option" >&2
            exit 3
        fi
        ;;
    r) rebuild=true ;;
    c) container="$OPTARG" ;;
    g) gpu=true ;;
    a) ros_ip="$OPTARG" ;;
    *) usage ;;
    esac
done

if [ -z "$tag" ]; then
    echo "Error: missing image tag name"
    usage
fi

echo -e "Rebuild : $rebuild\n"

# Check if the docker image with tag name $tag exists
if docker image inspect $tag >/dev/null 2>&1; then
    echo "Image '$tag' exists locally"
else
    # Else if it doesn't, ask the user if they want to build it or not
    read -p "Image '$tag' does not exist locally, do you want to build it [Y/n] : " confirm
    while [ "${confirm,,}" != "y" ] && [ "${confirm,,}" != "n" ]; do
        echo "$confirm"
        read -p "Image '$tag' does not exist locally, do you want to build it [Y/n] : " confirm
    done
    if [ "${confirm,,}" = "n" ]; then
        echo "Aborting..."
        exit 1
    fi
fi

if [ "$rebuild" = true ] || [ "${confirm,,}" = "y" ]; then
    echo -e "Build target : $target"
    if [ "$target" != "real" ] && [ "$target" != "simulation" ]; then
        echo -e "\nError : target '$target' should be either 'real' or 'simulation'"
        usage
    fi

    args=""

    # A ROS_IP has to be specified when using target 'real',
    # so that remote ROS nodes can contact us
    if [ "$ros_ip" ]; then
        args+="--build-arg IP=$ros_ip"
    fi

    # Build image and check for errors
    echo -e "Building image '$tag'... \n"
    docker build . -t "$tag" --target "$target" $args
    code=$?
    if [ $code -ne 0 ]; then
        echo "Error during build : exit code $code, aborting..."
        exit $code
    fi
fi

GPU_ARGS=""
if [ "$gpu"=true ]; then
    # Give access to all available GPUs to the container 
    # and run it in priviledged mode (so that it can access these GPUs)
    GPU_ARGS="--gpus all --privileged"
fi


echo -e "Running container '$container' with image '$tag'...\n"
# If you wish to add devices your host has access to to the container,
# you can add as many '--device' or '-d' followed by the path to the /dev/*
# you want to access inside the container BEFORE the the docker image tag
# example : docker run -ti --device /dev/video0 --name test rr100-sim
docker run --rm -ti \
    $GPU_ARGS \
    --net host \
    --env DISPLAY=$DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    --env XAUTHORITY=$XAUTH \
    --volume "$XAUTH:$XAUTH" \
    --volume /tmp/.X11-unix/:/tmp/.X11-unix/ \
    --volume $HOME/.gazebo:/root/.gazebo/ \
    --name $container $tag

## docker run options explanation : 
# --rm : deletes the container when exited
# -ti  : attaches a terminal to the container and puts it in interactive mode
# --net host : Mount the hosts machine network setup inside the container
# --env DISPLAY=$DISPLAY : Set the container display to the same display as the host
# --env QT_X11_NO_MITSHM=1 : Used to fix an error when running QT inside the container
# --env XAUTHORITY=$XAUTH : Use the same X authority as host
# --volume "$XAUTH:$XAUTH" : Mount your XAUTH inside the container
# -v /tmp/.X11-unix/:/tmp/.X11-unix/ : Mount the hosts X socket inside the container
# -v $(pwd)/.gazebo:/root/.gazebo/ : Mount the container's .gazebo cache directory 
    # to the host to avoid downloading gazebo 3D models on each build/new container
# --name $container $tag : Used to specify the container's name 
    # (useful for commands like docker exec <container-name> or docker cp)