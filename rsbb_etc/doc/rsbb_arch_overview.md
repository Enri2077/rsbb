RSBB arch overview
=================================================

The RSBB is a collection of ROS packages.
The packages communicate through ROS topics and services.

## Packages
* rsbb_etc: This package contains the configuration, documentation, the main launchfiles and resource material
* rsbb_refbox: This package provides the rqt GUI nodes and the core node. The core node communicates with the robot and with the other nodes of the RSBB, namely with the rqt GUI, the BmBox, the roah_devices and the record server nodes.
* rsbb_bmbox: This package provides the benchmark script server that executes the benchmark scripts (BmScripts) and communicates with the refbox. The bechmark scripts also communicate with the mocap node and external nodes.
* roah_devices: This package provides the node that interact with the Home Automation Devices.
* rsbb_record_server: This package provides the node that records the rosbags during the execution of the benchmarks.
* rsbb_mocap_optitrack: This package provides the node that receives the motion capture data and published it in the ROS framework.
* rsbb_benchmarking_messages: This package provides the messages and services used by the nodes for the communication between different packages.

## Interface

The interface for the communication between packages and nodes is composed by ROS services and topics.

The services and topics used to communicate between different packages are specified in rsbb_etc/msg.

`TODO: LIST OF SERVICES AND TOPICS.`


## Dependencies

`TODO: Dependencies`
