# docker run -it --rm -v "%cd%":/workspace mujoco-env

FROM python:3.10-slim

# 기본 패키지
RUN apt update && apt install -y \
    libgl1-mesa-glx libglfw3 libglew-dev libosmesa6-dev \
    wget unzip tree && \
    pip install --upgrade pip

# Python 패키지
COPY requirements.txt /tmp/
RUN pip install -r /tmp/requirements.txt

WORKDIR /workspace

CMD [ "bash" ]