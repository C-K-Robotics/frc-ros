.ONESHELL:
SHELL := /bin/bash
.DEFAULT_GOAL := build

.PHONY: clean
clean:
	@rm -rf build/ install/ log/ logs/ tests/

.PHONY: clean-docs
clean-docs:
	@rm -rf cross_reference/ docs_build/ docs_output/

.PHONY: purge
purge:
	@rm -rf build/ install/ log/ logs/ tests/ src/external

.PHONY: clean-test
clean-test:
	source ./tools/scripts/source_all.sh
	colcon test-result --delete-yes

.PHONY: vcs-import
vcs-import:
	@VCS_FILE="${VCS_FILE}"
	vcs import < ${VCS_FILE}

.PHONY: build-debug
build-debug:
	@PACKAGES="${PACKAGES}"
	source ./tools/scripts/source_all.sh
	if [ -z "$${PACKAGES}" ] ; then
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-up-to autonomy_launch basestation_launch tools_launch
	else
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-up-to ${PACKAGES}
	fi

.PHONY: build
build:
	@PACKAGES="${PACKAGES}"
	source ./tools/scripts/source_all.sh
	if [ -z "$${PACKAGES}" ] ; then
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to autonomy_launch basestation_launch tools_launch
	else
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to ${PACKAGES}
	fi

.PHONY: build-select
build-select:
	@PACKAGES="${PACKAGES}"
	source ./tools/scripts/source_all.sh
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select ${PACKAGES}

.PHONY: build-select-debug
build-select-debug:
	@PACKAGES="${PACKAGES}"
	source ./tools/scripts/source_all.sh
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select ${PACKAGES}

.PHONY: test-select
test-select:
	@PACKAGES="${PACKAGES}"
	source ./tools/scripts/source_all.sh
	if [ -z "$${PACKAGES}" ] ; then
		colcon test --packages-select $(shell cat .github/packages_for_test.txt); colcon test-result --verbose
	else
		colcon test --packages-select ${PACKAGES}; colcon test-result --verbose
	fi

.PHONY: test-cpp
test-cpp:
	source ./tools/scripts/source_all.sh
	ament_uncrustify ${PATHS}
	ament_cpplint ${PATHS}

.PHONY: reformat
reformat:
	source ./tools/scripts/source_all.sh
	autoflake --in-place --remove-unused-variables --remove-all-unused-imports --ignore-init-module-imports -r ${PATHS}
	black -l 99 ${PATHS}
	ament_uncrustify --reformat ${PATHS}

.PHONY: rosdep-install
rosdep-install:
	source ./tools/scripts/source_all.sh
	sudo apt update
	rosdep update
	rosdep install -y -r --rosdistro ${ROS_DISTRO} --ignore-src --from-paths src

.PHONY: rosdep-install-eol
rosdep-install-eol:
	source ./tools/scripts/source_all.sh
	sudo apt update
	rosdep update --include-eol-distros
	rosdep install -y -r --rosdistro ${ROS_DISTRO} --ignore-src --from-paths src

.PHONY: rosdep-install-list
rosdep-install-list:
	source ./tools/scripts/source_all.sh
	rosdep update --include-eol-distros
	rosdep install --as-root "apt:false pip:false" --simulate --reinstall --ignore-src -r -y --rosdistro ${ROS_DISTRO} --from-paths . | sort >> tools/image/ros-deps

.PHONY: build-docker-cpu-humble
build-docker-cpu-humble:
	@IMG_NAME=${IMG_NAME}
	@SSH_FILE_PATH=${SSH_FILE_PATH}
	eval $(ssh-agent)
	if [ -z "$${SSH_FILE_PATH}" ] ; then
		ssh-add ~/.ssh/id_ed25519
	else
		ssh-add ${SSH_FILE_PATH}
	fi
	DOCKER_BUILDKIT=1 docker build \
		--network=host \
		-f tools/image/Dockerfile \
		--target frc_image \
		--ssh default=${SSH_AUTH_SOCK} \
		--build-arg BASE_IMAGE=ubuntu:22.04 \
		--build-arg ROS_DISTRO=humble \
		--build-arg ROS_SOURCE=humble \
		--build-arg ROS_INSTALL=ros-install.sh\
		--build-arg SKIP_KEYS="🏎️" \
		--build-arg APT_FILE=apt-packages \
		--build-arg APT_GPU_FILE=empty-deps \
		--build-arg PIP_FILE=pip3-packages \
		--build-arg PIP_GPU_FILE=empty-deps \
		--build-arg PYTORCH_FILE=pytorch-cpu \
		--build-arg EXPORTS_SCRIPT=exports.sh \
		--build-arg EXPORTS_GPU_SCRIPT=empty-script.sh \
		--build-arg VCS_IMPORTS_SCRIPT=vcs-imports-humble.sh \
		--build-arg CUSTOM_INSTALL_FILE=custom-installs.sh \
		-t ${IMG_NAME} .

.PHONY: build-docker-cpu-humble-jetpack6
build-docker-cpu-humble-jetpack6:
	@IMG_NAME=${IMG_NAME}
	@SSH_FILE_PATH=${SSH_FILE_PATH}
	eval $(ssh-agent)
	if [ -z "$${SSH_FILE_PATH}" ] ; then
		ssh-add ~/.ssh/id_ed25519
	else
		ssh-add ${SSH_FILE_PATH}
	fi
	DOCKER_BUILDKIT=1 docker build \
		--network=host \
		-f tools/image/Dockerfile \
		--target frc_image_built \
		--ssh default=${SSH_AUTH_SOCK} \
		--build-arg BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r36.2.0 \
		--build-arg ROS_DISTRO=humble \
		--build-arg ROS_SOURCE="humble" \
		--build-arg ROS_INSTALL=ros-install.sh\
		--build-arg SKIP_KEYS="🏎️" \
		--build-arg APT_FILE=apt-packages-l4t \
		--build-arg APT_GPU_FILE=empty-deps \
		--build-arg PIP_FILE=pip3-packages \
		--build-arg PIP_GPU_FILE=empty-deps \
		--build-arg PYTORCH_FILE=pytorch-cpu \
		--build-arg EXPORTS_SCRIPT=exports.sh \
		--build-arg EXPORTS_GPU_SCRIPT=empty-script.sh \
		--build-arg VCS_IMPORTS_SCRIPT=vcs-imports-humble.sh \
		--build-arg CUSTOM_INSTALL_FILE=custom-installs-l4t.sh \
		--build-arg OPENCV_SCRIPT=opencv-480-install.sh \
		-t ${IMG_NAME} .

.PHONY: build-docker-gpu-humble-jetpack5
build-docker-gpu-humble-jetpack5:
	@IMG_NAME=${IMG_NAME}
	@SSH_FILE_PATH=${SSH_FILE_PATH}
	eval $(ssh-agent)
	if [ -z "$${SSH_FILE_PATH}" ] ; then
		ssh-add ~/.ssh/id_ed25519
	else
		ssh-add ${SSH_FILE_PATH}
	fi
	DOCKER_BUILDKIT=1 docker build \
		--network=host \
		-f tools/image/Dockerfile \
		--target frc_image \
		--ssh default=${SSH_AUTH_SOCK} \
		--build-arg BASE_IMAGE=dustynv/ros:humble-pytorch-l4t-r35.3.1 \
		--build-arg ROS_DISTRO=humble \
		--build-arg ROS_SOURCE="humble/install" \
		--build-arg ROS_INSTALL=ros-jetson-install.sh \
		--build-arg SKIP_KEYS="librange-v3-dev" \
		--build-arg APT_FILE=apt-packages-jetson \
		--build-arg APT_GPU_FILE=empty-deps \
		--build-arg PIP_FILE=pip3-packages-jetson \
		--build-arg PIP_GPU_FILE=pip3-packages-jetson-gpu \
		--build-arg PYTORCH_FILE=pytorch-gpu-jetson \
		--build-arg EXPORTS_SCRIPT=exports-jetson.sh \
		--build-arg EXPORTS_GPU_SCRIPT=empty-script.sh \
		--build-arg VCS_IMPORTS_SCRIPT=vcs-imports-humble-jetson.sh \
		--build-arg CUSTOM_INSTALL_FILE=custom-installs-jetson.sh \
		-t ${IMG_NAME} .

.PHONY: build-docker-gpu-humble-jetpack6
build-docker-gpu-humble-jetpack6:
	@IMG_NAME=${IMG_NAME}
	@SSH_FILE_PATH=${SSH_FILE_PATH}
	eval $(ssh-agent)
	if [ -z "$${SSH_FILE_PATH}" ] ; then
		ssh-add ~/.ssh/id_ed25519
	else
		ssh-add ${SSH_FILE_PATH}
	fi
	DOCKER_BUILDKIT=1 docker build \
		--network=host \
		-f tools/image/Dockerfile \
		--target frc_image \
		--ssh default=${SSH_AUTH_SOCK} \
		--build-arg BASE_IMAGE=dustynv/tensorrt:8.6-r36.2.0 \
		--build-arg ROS_DISTRO=humble \
		--build-arg ROS_SOURCE="humble" \
		--build-arg ROS_INSTALL=ros-install.sh\
		--build-arg SKIP_KEYS="🏎️" \
		--build-arg APT_FILE=apt-packages-l4t \
		--build-arg APT_GPU_FILE=empty-deps \
		--build-arg PIP_FILE=pip3-packages \
		--build-arg PIP_GPU_FILE=empty-deps \
		--build-arg PYTORCH_FILE=empty-deps \
		--build-arg EXPORTS_SCRIPT=exports.sh \
		--build-arg EXPORTS_GPU_SCRIPT=exports-gpu-cu122.sh \
		--build-arg VCS_IMPORTS_SCRIPT=vcs-imports-humble.sh \
		--build-arg CUSTOM_INSTALL_FILE=custom-installs-jp6.sh \
		-t ${IMG_NAME} .

.PHONY: build-docker-cu122-humble
build-docker-cu122-humble:
	@IMG_NAME=${IMG_NAME}
	@SSH_FILE_PATH=${SSH_FILE_PATH}
	eval $(ssh-agent)
	if [ -z "$${SSH_FILE_PATH}" ] ; then
		ssh-add ~/.ssh/id_ed25519
	else
		ssh-add ${SSH_FILE_PATH}
	fi
	DOCKER_BUILDKIT=1 docker build \
		--network=host \
		-f tools/image/Dockerfile \
		--target frc_image \
		--ssh default=${SSH_AUTH_SOCK} \
		--build-arg BASE_IMAGE=nvidia/cuda:12.2.2-devel-ubuntu22.04 \
		--build-arg ROS_DISTRO=humble \
		--build-arg ROS_SOURCE=humble \
		--build-arg ROS_INSTALL=ros-install.sh \
		--build-arg SKIP_KEYS="🏎️" \
		--build-arg APT_FILE=apt-packages \
		--build-arg APT_GPU_FILE=apt-packages-gpu-cu122 \
		--build-arg PIP_FILE=pip3-packages \
		--build-arg PIP_GPU_FILE=pip3-packages-gpu-cu122 \
		--build-arg PYTORCH_FILE=pytorch-gpu-cu122 \
		--build-arg EXPORTS_SCRIPT=exports-humble.sh \
		--build-arg EXPORTS_GPU_SCRIPT=exports-gpu-cu122.sh \
		--build-arg VCS_IMPORTS_SCRIPT=vcs-imports-humble.sh \
		--build-arg CUSTOM_INSTALL_FILE=custom-installs.sh \
		-t ${IMG_NAME} .

.PHONY: build-docker-cu118-humble
build-docker-cu118-humble:
	@IMG_NAME=${IMG_NAME}
	@SSH_FILE_PATH=${SSH_FILE_PATH}
	eval $(ssh-agent)
	if [ -z "$${SSH_FILE_PATH}" ] ; then
		ssh-add ~/.ssh/id_ed25519
	else
		ssh-add ${SSH_FILE_PATH}
	fi
	DOCKER_BUILDKIT=1 docker build \
		--network=host \
		-f tools/image/Dockerfile \
		--target frc_image \
		--ssh default=${SSH_AUTH_SOCK} \
		--build-arg BASE_IMAGE=nvidia/cuda:11.8.0-devel-ubuntu22.04 \
		--build-arg ROS_DISTRO=humble \
		--build-arg ROS_SOURCE=humble \
		--build-arg ROS_INSTALL=ros-install.sh \
		--build-arg SKIP_KEYS="🏎️" \
		--build-arg APT_FILE=apt-packages \
		--build-arg APT_GPU_FILE=apt-packages-gpu-cu118 \
		--build-arg PIP_FILE=pip3-packages \
		--build-arg PIP_GPU_FILE=pip3-packages-gpu-cu118 \
		--build-arg PYTORCH_FILE=pytorch-gpu-cu118 \
		--build-arg EXPORTS_SCRIPT=exports.sh \
		--build-arg EXPORTS_GPU_SCRIPT=exports-gpu-cu118.sh \
		--build-arg VCS_IMPORTS_SCRIPT=vcs-imports-humble.sh \
		--build-arg CUSTOM_INSTALL_FILE=custom-installs.sh \
		-t ${IMG_NAME} .

.PHONY: build-docker-cu122-humble
build-docker-cu122-humble:
	@IMG_NAME=${IMG_NAME}
	@SSH_FILE_PATH=${SSH_FILE_PATH}
	eval $(ssh-agent)
	if [ -z "$${SSH_FILE_PATH}" ] ; then
		ssh-add ~/.ssh/id_ed25519
	else
		ssh-add ${SSH_FILE_PATH}
	fi
	DOCKER_BUILDKIT=1 docker build \
		--network=host \
		-f tools/image/Dockerfile \
		--target frc_image \
		--ssh default=${SSH_AUTH_SOCK} \
		--build-arg BASE_IMAGE=nvidia/cuda:12.2.2-devel-ubuntu22.04 \
		--build-arg ROS_DISTRO=humble \
		--build-arg ROS_SOURCE=humble \
		--build-arg ROS_INSTALL=ros-install.sh \
		--build-arg SKIP_KEYS="🏎️" \
		--build-arg APT_FILE=apt-packages \
		--build-arg APT_GPU_FILE=apt-packages-gpu-cu122 \
		--build-arg PIP_FILE=pip3-packages \
		--build-arg PIP_GPU_FILE=pip3-packages-gpu-cu122 \
		--build-arg PYTORCH_FILE=pytorch-gpu-cu122 \
		--build-arg EXPORTS_SCRIPT=exports.sh \
		--build-arg EXPORTS_GPU_SCRIPT=exports-gpu-cu122.sh \
		--build-arg VCS_IMPORTS_SCRIPT=vcs-imports-humble.sh \
		--build-arg CUSTOM_INSTALL_FILE=custom-installs.sh \
		-t ${IMG_NAME} .

# -----------------------------------------------------------------------------
# Target: build-perception
# Description: Builds ROS2 perception packages using colcon build with Release
# 	build type. This requires the external perception packages to have already
# 	been imported with vcs-import.
# Parameters:
# 	PACKAGES: A list of packages to build. If not provided, all perception
#		packages will be built.
# Errors:
# 	- Exits if the yolov8 repo is not found
#   - Exits if the IAC_Perception repo is not found
# Usage: make build-perception PACKAGES="package1..."
# -----------------------------------------------------------------------------
.PHONY: build-perception
build-perception:
	@PACKAGES="${PACKAGES}"
	if [ ! -d "./src/external/perception/yolov8" ]; then
		echo "Error: yolov8 repo not found. Please import the yolov8 repo with"
		echo "vcs-import"
		exit 1
	fi
	if [ ! -d "./src/external/IAC_Perception" ]; then
		echo "Error: IAC_Perception directory not found."
		exit 1
	fi
	source ./tools/scripts/source_all.sh
	if [ -z "$${PACKAGES}" ] ; then
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to \
			perception_launch
	else
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to ${PACKAGES}
	fi

# -----------------------------------------------------------------------------
# Target: build-perception-debug
# Description: Builds ROS2 perception packages using colcon build with Debug
# 	build type to create debuggable binaries. This requires the external
#	perception packages to have already been imported with vcs-import.
# Parameters:
# 	PACKAGES: A list of packages to build. If not provided, all perception
#		packages will be built.
# Errors:
# 	- Exits if the yolov8 repo is not found
#   - Exits if the IAC_Perception repo is not found
# Usage: make build-perception-debug PACKAGES="package1..."
# -----------------------------------------------------------------------------
.PHONY: build-perception-debug
build-perception-debug:
	@PACKAGES="${PACKAGES}"
	if [ ! -d "./src/external/perception/yolov8" ]; then
		echo "Error: yolov8 repo not found. Please import the yolov8 repo with"
		echo "vcs-import"
		exit 1
	fi
	if [ ! -d "./src/external/IAC_Perception" ]; then
		echo "Error: IAC_Perception directory not found."
		exit 1
	fi
	source ./tools/scripts/source_all.sh
	if [ -z "$${PACKAGES}" ] ; then
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-up-to \
			perception_launch 
	else
		colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-up-to ${PACKAGES}
	fi

# -----------------------------------------------------------------------------
# Target: install-opencv-cuda
# Description: Builds and installs OpenCV with CUDA support system-wide using
# 	yolov8's OpenCV install script. This requires the yolov8 repo to have
# 	already been imported with vcs-import.
# Parameters: None
# Exceptions:
# 	- Exits if the yolov8 repo is not found
#   - Exits if the CUDA toolkit version being used on the system is not 11.8
# Usage: make install-opencv-cuda
# -----------------------------------------------------------------------------
.PHONY: install-opencv-cuda
install-opencv-cuda:
	@if [ ! -d "./src/external/perception/yolov8" ]; then
		echo "Error: yolov8 repo not found. Please import the yolov8 repo with"
		echo "vcs-import"
		exit 1
	fi
	source ./tools/cuda/cuda_version_tools.sh
	enforce_cuda_version 11.8
	source ./src/external/perception/yolov8/src/yolov8/scripts/install_opencv.sh

# -----------------------------------------------------------------------------
# Target: uninstall-opencv-cuda
# Description: Uninstalls OpenCV with CUDA support system-wide using yolov8's
# 	OpenCV uninstall script. This requires the yolov8 repo to have already been
#	imported with vcs-import.
# Parameters: None
# Usage: make uninstall-opencv-cuda
# -----------------------------------------------------------------------------
.PHONY: uninstall-opencv-cuda
uninstall-opencv-cuda:
	@if [ ! -d "./src/external/perception/yolov8" ]; then
		echo "Error: yolov8 repo not found. Please import the yolov8 repo with"
		echo "vcs-import"
		exit 1
	fi
	source ./src/external/perception/yolov8/src/yolov8/scripts/uninstall_opencv.sh

.PHONY: session
session:
	@CONT_NAME="${CONT_NAME}"
	@IMG_NAME="${IMG_NAME}"
	@RUNTIME="${RUNTIME}"
	@INTERACTIVE="${INTERACTIVE}"
	if [ "${INTERACTIVE}" == "true" ]; then
		INTERACTIVE_FLAGS="-it"
		ENTRYPOINT="/bin/bash"
	else
		INTERACTIVE_FLAGS="-d"
		ENTRYPOINT="tail -f /dev/null"
	fi
	if [ "${RUNTIME}" = "nvidia" ]; then
		echo "RUNTIME is set to nvidia"
		xhost +
		docker run \
			--name ${CONT_NAME} \
			--runtime nvidia \
			$${INTERACTIVE_FLAGS} \
			--rm \
			--privileged \
			--net=host \
			--gpus all \
			-e NVIDIA_DRIVER_CAPABILITIES=all \
			-e DISPLAY=${DISPLAY} \
			-v /dev/bus/usb:/dev/bus/usb \
			--device-cgroup-rule='c 189:* rmw' \
			--device /dev/video0 \
			--volume='/dev/input:/dev/input' \
			--volume='${HOME}/.Xauthority:/root/.Xauthority:rw' \
			--volume='/tmp/.X11-unix/:/tmp/.X11-unix' \
			--volume='${PWD}:/opt/frc-ros' \
			${IMG_NAME} $${ENTRYPOINT}
	else
		xhost +
		docker run \
			--name ${CONT_NAME} \
			$${INTERACTIVE_FLAGS} \
			--rm \
			--privileged \
			--net=host \
			-e DISPLAY=${DISPLAY} \
			-v /dev/bus/usb:/dev/bus/usb \
			--device-cgroup-rule='c 189:* rmw' \
			--device /dev/video0 \
			--volume='/dev/input:/dev/input' \
			--volume='${HOME}/.Xauthority:/root/.Xauthority:rw' \
			--volume='/tmp/.X11-unix/:/tmp/.X11-unix' \
			--volume='${PWD}:/opt/frc-ros' \
			${IMG_NAME} $${ENTRYPOINT}
	fi

.PHONY: join-session
join-session:
	@CONT_NAME="${CONT_NAME}"
	docker exec -it ${CONT_NAME} /bin/bash

.PHONY: image-update
image-update:
	docker pull ghcr.io/ucsd-ecemae-148/donkeycontainer:ros

.PHONY: docker-build
docker-build:
	DOCKER_BUILDKIT=0 docker build -t frc-ros:test -f .docker_utils/Dockerfile.arm .