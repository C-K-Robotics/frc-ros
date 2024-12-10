#!/bin/bash
# Description: Defines bash functions check and enforces the version of CUDA

# -----------------------------------------------------------------------------
# Function: get_cuda_version
# Description: Uses the nvcc command to get the version of CUDA being used by
#   the system and sets the CUDA_VERSION environment variable to the version
#   number.
# Exceptions:
#   - Exits if the nvcc command is not found.
# Usage: get_cuda_version
# -----------------------------------------------------------------------------
get_cuda_version() {
    if command -v nvcc &> /dev/null; then
        # Get the CUDA version
        CUDA_VERSION=$(nvcc --version | grep -oP 'release \K\d+\.\d+')
        export CUDA_VERSION
    else
        echo "nvcc command not found. Is the CUDA toolkit installed?"
        exit 1
    fi
}

# -----------------------------------------------------------------------------
# Function: enforce_cuda_version
# Description: Enforces the use of a specific version of CUDA by checking the
#   version of CUDA being used by the system and exiting if it does not match
# Exceptions:
#   - Exits if the nvcc command is not found.
#   - Exits if the specified version number is not provided.
#   - Exits if the CUDA version does not match the specified version number.
# Usage: get_cuda_version <version_number>
# Returns:
#   0 - If the CUDA version matches the specified version
# -----------------------------------------------------------------------------
enforce_cuda_version() {
    # Check $1 is provided
    if [ -z "$1" ]; then
        echo "Usage: enforce_cuda_version <version_number>"
        exit 1
    fi
    # Get the CUDA version
    get_cuda_version

    # Check if the CUDA version matches the specified version
    if [ "$CUDA_VERSION" != "$1" ]; then
        echo "CUDA version $1 is required but version $CUDA_VERSION is being used."
        exit 1
    else
        echo "CUDA version $1 is being used."
        return 0
    fi
}
