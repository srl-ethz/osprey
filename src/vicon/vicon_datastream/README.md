# vicon-datastream-sdk
inofficial VICON DataStream SDK with cmake support

Can be compiled on mac,windows and ubuntu. No dynamic linking (unlike the official sdk)


## Building

````
mkdir build
cd build
cmake ..
make
````

## Testing

Run

````
cd build
./vicon_datastream_app/vicon_datastreamer_cleaned 10.10.10.5
````

Substitute the IP Address with the one your vicon software is sending the UDP packets to

where vicon is the IP/hostname of the vicon machine.

## Notes

Currently this only builds the C++ SDK and associated test application.