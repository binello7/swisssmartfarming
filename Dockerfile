# syntax=docker/dockerfile:1
FROM ubuntu:bionic
# RUN apk add --no-cache python g++ make
RUN apt update && \
    apt install -y \
    build-essential \
    git \
    libboost-python-dev \
    libexiv2-dev \
    libimage-exiftool-perl \
    python-all-dev \
    python-pip \
    python3-venv
RUN mkdir -p /app
COPY requirements_py3.txt /app
WORKDIR /app
RUN pip install -r requirements_py3.txt
# COPY . .
# RUN yarn install --production
CMD ["/usr/bin/sh", "echo test"]
