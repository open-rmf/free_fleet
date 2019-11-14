# generating the CycloneDDS message types
For convenience, the generation of FreeFleet.idl has been done offline. To
re-create it, use the `dds_idlc` generator that is built with CycloneDDS:
```
./dds_idlc -allstructs FreeFleet.idl
```
