echo export CUDA_HOME="/usr/local/cuda-12.2" >> /etc/bash.bashrc
echo export PATH="/usr/local/cuda-12.2/bin:${PATH}" >> /etc/bash.bashrc
echo export LD_LIBRARY_PATH="/usr/local/cuda-12.2/lib64:${LD_LIBRARY_PATH}" >> /etc/bash.bashrc
echo export NVIDIA_VISIBLE_DEVICES="all" >> /etc/bash.bashrc
echo export NVIDIA_DRIVER_CAPABILITIES="all" >> /etc/bash.bashrc