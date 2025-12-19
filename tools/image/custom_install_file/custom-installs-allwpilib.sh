#!/bin/sh
apt update
apt install -y openjdk-17-jdk

git clone https://github.com/wpilibsuite/allwpilib.git
cd allwpilib && ./gradlew installRoboRioToolchain && ./gradlew :wpilibc:build --build-cache
