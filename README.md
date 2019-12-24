![](https://github.com/osrf/free_fleet/workflows/build/badge.svg)

This repository is under active development. Things will be quite unstable
for a while. Please open an issue ticket on this repo if you have problems.
Cheers.

# Overview
Welcome to `free_fleet`, an open-source robot fleet management system.
Sometimes it is called the "Fun Free Fleet For Friends" (F5).

# Requirements

ROS 1 Melodic and ROS 2 Eloquent need to be installed, since the ROS 1 client
uses DDS implementations packaged as part of the ROS 2 project.

The build scripts assume that ROS 2 Eloquent is installed to the canonical
location in `/opt/ros/eloquent`

# Documentation and notes

* [Generating the CycloneDDS message types](docs/msgs.md)

* [Building the ROS 1 Client](docs/clients.md)

* [Running tests on the ROS 1 Client](docs/clients_test.md)
