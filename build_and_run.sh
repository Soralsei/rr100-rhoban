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
# replace long arguments
for arg in "$@"; do
    case "$arg" in
    --help) args+=(-h) ;;
    --image-tag) args+=(-i) ;;
    --container-name) args+=(-c) ;;
    --rebuild) args+=(-r) ;;
    --target) args+=(-t) ;;
    --gpu) args+=(-g) ;;
    --ip-address) args+=(-a);;
    *) args+=("$arg") ;;
    esac
done
set -- "${args[@]}"

builder="builder"
target="real"
tag=""
rebuild=false
gpu=false
container="ros"
ros_ip=""

while getopts t:b:hi:rgc:a: option; do
    : "$option" "$OPTARG"
    case $option in
    i)
        tag="$OPTARG"
        if [[ ${OPTARG:0:1} == '-' ]]; then
            echo "Invalid value '$OPTARG' given to -$option" >&2
            exit 3
        fi
        ;;
    t)
        target="$OPTARG"
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

if docker image inspect $tag >/dev/null 2>&1; then
    echo "Image '$tag' exists locally"
else
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

    if [ "$ros_ip" ]; then
        args+="--build-arg IP=$ros_ip"
    fi

    echo -e "Building image '$tag'... \n"
    docker build . -t "$tag" --target "$target" $args
    code=$?
    if [ $code -ne 0 ]; then
        echo "Error during build : exit code $code, aborting..."
        exit $code
    fi
fi

echo -e "Running container '$container' with image '$tag'...\n"
GPU_ARGS=""
if [ "$gpu"=true ]; then
    GPU_ARGS="--gpus all --privileged"
fi
docker run --rm -ti \
    $GPU_ARGS \
    --net host \
    --env DISPLAY=$DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    --env XAUTHORITY=$XAUTH \
    --volume "$XAUTH:$XAUTH" \
    -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
    -v $(pwd)/.gazebo:/root/.gazebo/ \
    --name $container $tag
