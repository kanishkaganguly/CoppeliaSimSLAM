#!/bin/bash

function build () {
    DOCKER_BUILDKIT=1 docker build -t vrep_image -f vrep_docker.Dockerfile .
}

# See: https://unix.stackexchange.com/questions/462701/how-to-get-the-current-path-without-last-folder-in-a-script
function run (){
    if [ "$(docker container ls -a --format '{{split .Names ":"}}' | grep -c VREP)" == 1 ]; then
        echo 'Deleting existing container...'
        docker container rm --force VREPContainer
    fi
    docker run -d --name VREPContainer -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v ${PWD%/*/*}:/home/user/workspace/src:rw --net=host --privileged --runtime=nvidia vrep_image:latest
}

function execute (){
    if [ "$(docker container ls -a --format '{{split .Names ":"}}' | grep -c VREP)" == 0 ]; then
        echo 'Container is not running, exiting...'
    fi
    docker exec -it -u user VREPContainer /bin/bash
}

if [[ $1 == "build" ]]; then
    build
elif [[ $1 == "run" ]]; then
    run
elif [[ $1 == "exec" ]]; then
    execute
else
    printf "Docker Setup for VREP\n"
    printf "Usage:\n"
    printf "\t$0 build \t[Build the dockerfile into an image]\n"
    printf "\t$0 run \t\t[Run the created image]\n"
    printf "\t$0 execute \t[Execute bash in the container]\n"
fi