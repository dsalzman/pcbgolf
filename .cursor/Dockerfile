FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive
ARG FREEROUTING_VERSION=2.4.1
ARG FREEROUTING_SHA256=251101c3eeac22d7e7dfcf6796603279e5d1000283eb82d8f093780f7afc6aa9

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        git \
        openjdk-25-jre-headless \
        python3 \
        software-properties-common \
        sudo \
    && add-apt-repository --yes ppa:kicad/kicad-10.0-releases \
    && apt-get update \
    && apt-get install -y --no-install-recommends kicad \
    && rm -rf /var/lib/apt/lists/*

RUN install -d /opt/freerouting \
    && curl --fail --location --retry 4 \
        "https://github.com/freerouting/freerouting/releases/download/v${FREEROUTING_VERSION}/freerouting-${FREEROUTING_VERSION}.jar" \
        --output "/opt/freerouting/freerouting-${FREEROUTING_VERSION}.jar" \
    && echo "${FREEROUTING_SHA256}  /opt/freerouting/freerouting-${FREEROUTING_VERSION}.jar" \
        | sha256sum --check -

RUN useradd --create-home --shell /bin/bash ubuntu \
    && usermod --append --groups sudo ubuntu \
    && printf 'ubuntu ALL=(ALL) NOPASSWD:ALL\n' > /etc/sudoers.d/ubuntu \
    && chmod 0440 /etc/sudoers.d/ubuntu

ENV FREEROUTING_JAR=/opt/freerouting/freerouting-2.4.1.jar
ENV PYTHONUNBUFFERED=1

USER ubuntu
WORKDIR /workspace
