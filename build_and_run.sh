#!/bin/bash

# Default variables initialization
builder="builder"
target="real"
tag=""
rebuild=false
gpu=false
container="ros"
ros_ip=""

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

confirmed=

confirm() {
    read -p "$1 [Y/n] : " answer
    while [ "${answer,,}" != "y" ] && [ "${answer,,}" != "n" ]; do
        read -p "$1 [Y/n] : " answer
    done
    if [ "${answer,,}" = "y" ]; then
        confirmed=true
    else
        confirmed=false
    fi
}

compare_builds() {
    changed=false

    mkdir -p .temp/
    # We should probably check that the image is built with the same Dockerfile
    # but for now, just assume the user won't give the name of a completely unrelated image
    (docker create --name files "$tag" && docker cp files:/opt/ros/rr100_ws/ .temp/ && docker container rm files) > /dev/null
    cd .temp/rr100_ws

    if [ ! -f "TARGET_${target^^}" ]; then
        echo "Target changed between builds"
        changed=true
    else
        # Compute the SHA256 checksum of the files copied to the Docker image as well as the Dockerfile
        ## Uses perl to find raw COPY instructions (ie. copying build context files/dirs into the image) in the Dockerfile
        ## And get their names.
        ## Then, find all files in the extracted file/directory names and pipe their contents into sha256sum to get the sum
        checksum_host=$((echo Dockerfile ; cat Dockerfile | perl -lne 'print $1 if m/^COPY (?!--)([^ ]+) .*/') | xargs -I@ sh -c "find @ -type f" | sort | xargs cat | sha256sum | cut -d' ' -f1)
        echo "Build context hash : $checksum_host"

        # Same as above
        checksum_container=$((echo Dockerfile ; cat Dockerfile | perl -lne 'print $1 if m/^COPY (?!--)([^ ]+) .*/') | xargs -I@ sh -c "find @ -type f" | sort | xargs cat | sha256sum | cut -d' ' -f1)
        echo "Container hash : $checksum_container"
        
        [ "$checksum_host" != "$checksum_container" ] && changed=true || changed=false
    fi

    cd ../..
    rm -rf .temp/

    if [ "$changed" = true ]; then
        confirm "Build context changed since last build of image, do you want to rebuild it"
        if [ "$confirmed" = true ]; then
            rebuild=true
        fi
    fi
}

# Show usage if no arguments passed
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
if [ "$rebuild" = false ]; then
    # Check if the docker image with tag name $tag exists
    # Explanation : 
    # docker image inspect $tag >/dev/null 2>&1 : 
    #### - redirect ALL output (stdout AND stderr) to /dev/null to discard it
    # if no error exit code, will execute echo "..."; compare_build
    # else ask if user wants to build new image
    if docker image inspect $tag >/dev/null 2>&1; then
        echo "Image '$tag' exists locally"
        compare_builds
    else
        # else if it doesn't, ask the user if they want to build it or not
        confirm "Image '$tag' does not exist locally, do you want to build it"
        if [ "$confirmed" = false ]; then
            echo "Aborting..."
            exit 1
        fi
        rebuild=true
    fi
fi

# If rebuild needed
if [ "$rebuild" = true ]; then
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
    --name $container \
    $tag

## docker run options explanation : 
# --rm : deletes the container when exited
# -ti  : attaches a terminal to the container and puts it in interactive mode
# --net host : Use the host machine's network stack inside the container
# --env DISPLAY=$DISPLAY : Set the container display to the same display as the host
# --env QT_X11_NO_MITSHM=1 : Used to fix an error when running QT inside the container
# --env XAUTHORITY=$XAUTH : Use the same X authority as host
# --volume "$XAUTH:$XAUTH" : Mount your XAUTH inside the container
# -volume /tmp/.X11-unix/:/tmp/.X11-unix/ : Mount the hosts X socket inside the container
# -volume $HOME/.gazebo:/root/.gazebo/ : Mount the container's .gazebo cache directory 
    # to the host to avoid downloading gazebo 3D models on each build/new container
# --name $container : Used to specify the container's name 
    # (useful for commands like docker exec <container-name> or docker cp)