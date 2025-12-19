ARG BASE_IMAGE

FROM $BASE_IMAGE as base
LABEL maintainer="Winston Chou <winston.h.chou@gmail.com>"
SHELL ["/bin/bash", "-o", "pipefail", "-ic"]

ARG APT_FILE
ARG APT_GPU_FILE
ARG PIP_FILE
ARG PIP_GPU_FILE
ARG PYTORCH_FILE
ARG CUSTOM_INSTALL_FILE

## Install apt packages
RUN apt update && DEBIAN_FRONTEND=noninteractive apt -y install --no-install-recommends \
  ca-certificates \
  && apt clean \
  && rm -rf /var/lib/apt/lists/*

RUN apt update && DEBIAN_FRONTEND=noninteractive apt -y install --no-install-recommends \
  software-properties-common \
  && apt clean \
  && rm -rf /var/lib/apt/lists/*

COPY tools/image/apt_file/${APT_FILE} /frc_tmp/${APT_FILE}

RUN apt update && DEBIAN_FRONTEND=noninteractive apt -y install --no-install-recommends \
        $(cat /frc_tmp/${APT_FILE} | cut -d# -f1) && \
    apt clean && \
    rm -rf /var/lib/apt/lists/* /frc_tmp/${APT_FILE}

COPY tools/image/apt_gpu_file/${APT_GPU_FILE} /frc_tmp/${APT_GPU_FILE}
RUN apt update && DEBIAN_FRONTEND=noninteractive apt -y install --no-install-recommends \
        $(cat /frc_tmp/${APT_GPU_FILE} | cut -d# -f1) && \
    apt clean && \
    rm -rf /var/lib/apt/lists/* /frc_tmp/${APT_GPU_FILE}

## Add GitHub to known hosts for private repositories
RUN mkdir -p ~/.ssh \
  && ssh-keyscan github.com >> ~/.ssh/known_hosts \
  && ssh-keyscan gitlab.com >> ~/.ssh/known_hosts

RUN python3 -m pip install --upgrade pip

COPY tools/image/pip_file/${PIP_FILE} /frc_tmp/${PIP_FILE}
RUN DEBIAN_FRONTEND=noninteractive pip3 install -U --no-cache --ignore-installed -r /frc_tmp/${PIP_FILE} && \
    rm -rf /frc_tmp/${PIP_FILE}

COPY tools/image/pip_gpu_file/${PIP_GPU_FILE} /frc_tmp/${PIP_GPU_FILE}
RUN DEBIAN_FRONTEND=noninteractive pip3 install -U --no-cache --ignore-installed -r /frc_tmp/${PIP_GPU_FILE} && \
    rm -rf /frc_tmp/${PIP_GPU_FILE}

COPY tools/image/pytorch_file/${PYTORCH_FILE} /frc_tmp/${PYTORCH_FILE}
RUN DEBIAN_FRONTEND=noninteractive pip3 install -U --no-cache --ignore-installed -r /frc_tmp/${PYTORCH_FILE} && \
    rm -rf /frc_tmp/${PYTORCH_FILE}

RUN rm -rf /frc_tmp

FROM base as base_custom

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}

COPY tools/image/custom_install_file/${CUSTOM_INSTALL_FILE} /frc_tmp/${CUSTOM_INSTALL_FILE}
RUN --mount=type=ssh \
    DEBIAN_FRONTEND=noninteractive /bin/sh /frc_tmp/${CUSTOM_INSTALL_FILE} && \
    rm -rf /frc_tmp/${CUSTOM_INSTALL_FILE}

COPY tools/image/custom_install_file/custom-installs-allwpilib.sh /frc_tmp/custom-installs-allwpilib.sh
RUN --mount=type=ssh \
    DEBIAN_FRONTEND=noninteractive /bin/sh /frc_tmp/custom-installs-allwpilib.sh && \
    rm -rf /frc_tmp/custom-installs-allwpilib.sh

FROM base_custom as base_ros

ARG ROS_DISTRO
ARG ROS_INSTALL

# Install Ros Distro
COPY tools/image/ros_install/${ROS_INSTALL} /frc_tmp/${ROS_INSTALL}
RUN DEBIAN_FRONTEND=noninteractive /bin/bash /frc_tmp/${ROS_INSTALL} ${ROS_DISTRO} && \
    rm -rf /frc_tmp/${ROS_INSTALL}

FROM base_ros as frc_ros_deps_generator

ARG ROS_DISTRO
ARG ROS_SOURCE
ARG VCS_IMPORTS_SCRIPT
ARG SKIP_KEYS

COPY src/ /workspace/src
COPY *.repos /workspace
COPY tools/image/vcs_imports_script/${VCS_IMPORTS_SCRIPT} /workspace/${VCS_IMPORTS_SCRIPT}
COPY tools/image/skip_keys/${SKIP_KEYS} /workspace/${SKIP_KEYS}

RUN --mount=type=ssh \
    source /opt/ros/${ROS_SOURCE}/setup.bash && \
    apt update && \
    rosdep update --include-eol-distros && \
    cd /workspace && \
    rm -rf src/external && \
    DEBIAN_FRONTEND=noninteractive ./${VCS_IMPORTS_SCRIPT} && \
    DEBIAN_FRONTEND=noninteractive rosdep install --as-root "apt:false pip:false" --simulate --reinstall -r -y --ignore-src --rosdistro ${ROS_DISTRO} --skip-keys $(cat /workspace/${SKIP_KEYS} | cut -d# -f1) --from-paths . | sort >> /ros-deps && \
    cd / && \
    rm -rf /workspace

FROM base_ros as frc_ros_deps

ARG ROS_DISTRO

COPY --from=frc_ros_deps_generator /ros-deps .

RUN cat /ros-deps

RUN apt update && \
    rosdep update --include-eol-distros && \
    DEBIAN_FRONTEND=noninteractive /bin/bash ros-deps && \
    apt clean && \
    rm -rf /var/lib/apt/lists/* /ros-deps

FROM frc_ros_deps as frc_image

ARG ROS_DISTRO
ARG ROS_SOURCE
ARG EXPORTS_SCRIPT
ARG EXPORTS_GPU_SCRIPT

ENV ROS_DISTRO=${ROS_DISTRO}

COPY tools/image/exports_script/${EXPORTS_SCRIPT} /frc_tmp/${EXPORTS_SCRIPT}
COPY tools/image/exports_gpu_script/${EXPORTS_GPU_SCRIPT} /frc_tmp/${EXPORTS_GPU_SCRIPT}
RUN  cd /frc_tmp && \
    ./${EXPORTS_GPU_SCRIPT} && \
    ./${EXPORTS_SCRIPT} && \
    rm -rf /frc_tmp/

COPY tools/image/cyclonedds.xml /etc/cyclone/cyclonedds.xml

FROM frc_image as frc_image_built

ARG VCS_IMPORTS_SCRIPT
ARG RC_BUILD_TARGET
ARG ROS_SOURCE

COPY src/ /opt/frc-ros/src
COPY *.repos /opt/frc-ros
COPY tools/ /opt/frc-ros/tools
COPY Makefile /opt/frc-ros/Makefile

RUN --mount=type=ssh \
    --mount=type=cache,target=/opt/frc-ros/build \
    source /opt/ros/${ROS_SOURCE}/setup.bash && \
    cd /opt/frc-ros && \
    rm -rf src/external && \
    DEBIAN_FRONTEND=noninteractive ./tools/image/vcs_imports_script/${VCS_IMPORTS_SCRIPT} && \
    export PATH="/opt/cmake-install/bin:${PATH}" && \
    export CMAKE_PREFIX_PATH="/opt/cmake-install:${CMAKE_PREFIX_PATH}" && \
    export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/usr/local/lib" && \
    DEBIAN_FRONTEND=noninteractive make ${RC_BUILD_TARGET} && \
    rm -rf src

RUN echo "source /opt/frc-ros/install/setup.bash" >> /etc/bash.bashrc

CMD ["/bin/bash", "-l", "-c", "exec /bin/bash"]

