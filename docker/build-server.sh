#!/bin/bash
BASE_IMAGE_TAG=noetic
IMAGE_NAME=iam_project_harshit

SERVE_REMOTE=false
REMOTE_SSH_PORT=3472


IIWA_TOOLKIT_BRANCH="feature_ns_inertial_control"
INERTIA=false

HELP_MESSAGE="Usage: ./build-server.sh [-b|--branch branch] [-r] [-v] [-s]
Build a Docker container for remote development and/or running unittests.
Options:
  --base-tag               The tag of ros2-control-libraries image.

  -b|--branch-iiwa-toolkit Branch name for the iiwa-toolkit (either feature_ns_inertial_control or feature_ns_full_inertia) (ONLY IF INERTIA FLAG IS USED!)

  -i|--inertia             Build docker based on iiwa_ros feature/inertia and use iiwa_toolkit controller

  -r, --rebuild            Rebuild the image with no cache.

  -v, --verbose            Show all the output of the Docker
                           build process

  -s, --serve              Start the remote development server.

  -h, --help               Show this help message.
"

BUILD_FLAGS=()
while [ "$#" -gt 0 ]; do
  case "$1" in
    --base-tag) BASE_IMAGE_TAG=$2; shift 2;;
    -b|--branch-iiwa-toolkit) IIWA_TOOLKIT_BRANCH=$2; shift 2;;
    -i|--inertia) INERTIA=true; shift 1;;
    -r|--rebuild) BUILD_FLAGS+=(--no-cache); shift 1;;
    -v|--verbose) BUILD_FLAGS+=(--progress=plain); shift 1;;
    -s|--serve) SERVE_REMOTE=true ; shift ;;
    -h|--help) echo "${HELP_MESSAGE}"; exit 0;;
    *) echo "Unknown option: $1" >&2; echo "${HELP_MESSAGE}"; exit 1;;
  esac
done

# docker pull magnificentmonkey/iiwa-ros
BUILD_FLAGS+=(--build-arg BASE_IMAGE_TAG="${BASE_IMAGE_TAG}")
BUILD_FLAGS+=(--build-arg IIWA_TOOLKIT_BRANCH="${IIWA_TOOLKIT_BRANCH}")
BUILD_FLAGS+=(-t "${IMAGE_NAME}:${BASE_IMAGE_TAG}")
BUILD_FLAGS+=(--build-arg HOST_GID=$(id -g))   # Pass the correct GID to avoid issues with mounted volumes

BUILD_FLAGS+=(--ssh default="${SSH_AUTH_SOCK}") # Pass git ssh key to be able to pull


if [ "${INERTIA}" = true ]; then
  DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" -f ./docker/Dockerfile_inertia .
else
DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" -f ./docker/Dockerfile .
fi

if [ "${SERVE_REMOTE}" = true ]; then
  aica-docker server "${IMAGE_NAME}:${BASE_IMAGE_TAG}" -u ros -p "${REMOTE_SSH_PORT}"
fi
