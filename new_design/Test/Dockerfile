FROM nvcr.io/nvidia/l4t-base:r32.7.1

RUN apt-get update
RUN apt-get install -y liblapack-dev libblas-dev gfortran libfreetype-dev libopenblas-dev libopenmpi-dev libjpeg-dev zlib1g-dev
RUN apt-get install -y python3-pip
RUN apt-get install -y python3.12-venv

WORKDIR /src
RUN python3 -m venv /app/venv
ENV PATH="/app/venv/bin:$PATH"

RUN pip3 install pandas
RUN pip3 install numpy
RUN pip3 install pyyaml
RUN pip3 install scipy
RUN pip3 install psutil
RUN pip3 install pillow
RUN pip3 install tqdm
RUN pip3 install imutils

RUN export PATH=/usr/local/cuda-10.2/bin${PATH:+:${PATH}}
RUN export LD_LIBRARY_PATH=/usr/local/cuda-10.2/lib64:$LD_LIBRARY_PATH
RUN apt-get install -y wget