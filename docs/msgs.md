# Generating the CycloneDDS message types

For convenience, the generation of FreeFleet.idl has been done offline. To
re-create it, use the `dds_idlc` generator that is built with CycloneDDS:

```bash
./dds_idlc -allstructs FreeFleet.idl
```

## Finding the elusive executable

Follow the instructions to build [rmw_cyclonedds_cpp](https://github.com/ros2/rmw_cyclonedds). Once built, the executable can be found at the path `ros2_ws/build/cyclonedds/src/idlc/dds_idlc`.
