RSBB arch overview
=================================================

The RSBB is a collection of ROS packages.
The packages communicate through ROS topics and services.
`TODO: cont (robot, devices, mocap)`

## Packages

* rsbb_etc: This package contains the configuration, documentation, the main launchfiles and resource material
* rsbb_refbox: This package provides the rqt GUI nodes and the core node. The core node communicates with the robot and with the other nodes of the RSBB, namely with the rqt GUI, the BmBox, the roah_devices and the record server nodes.
* rsbb_bmbox: This package provides the benchmark script server that executes the benchmark scripts (BmScripts) and communicates with the refbox. The bechmark scripts also communicate with the mocap node and external nodes.
* rsbb_utils: This package provides python scripts used to acquire data that is used by benchmark scripts.
* roah_devices: This package provides the node that interact with the Home Automation Devices.
* rsbb_record_server: This package provides the node that records the rosbags during the execution of the benchmarks.
* rsbb_mocap_optitrack: This package provides the node that receives the motion capture data and published it in the ROS framework.
* rsbb_benchmarking_messages: This package provides the messages and services used by the nodes for the communication between different packages.

## Node Architecture

`TODO: insert arch UML with all RSBB nodes`


## Interface and Communication

### Internal communication

The interface for the communication between the RSBB nodes is composed by ROS services and topics.

The services and topics used to communicate between different packages are specified in rsbb_benchmarking_messages/msg.

`TODO cont`

### Robot communication

#### Network

`TODO: insert network graph`

The RSBB uses the protobuf_comm library for communication.
All communication uses UDP, over two types of channels.
A single public channel uses UDP broadcast to communicate with all robots at the same time.
Multiple private channels are used to communicate with a single robot while it is running a benchmark.
Private channels use UDP unicast.
The robots and the RSBB must be on the same network, and the rsbb_broadcast_address parameter must be the broadcast address of the network.
The public channel is set up at the port specified in the rsbb_port parameter.
The private channels use rsbb_port+1, rsbb_port+2 and so on without reusing.

Note that, the RSBB and the robot can not be executed on the same computer.

#### Public channel

The public channel is used to transmit information that is relevant to all robots unencrypted.
The RSBB transmits the RoahRsbbBeacon every second, containing:
* Information about which robots are active in benchmarks.

Robots listed here should set up private channels for further information.
* The full state of the home devices.
* The full state of the tablet.

Active robots should also transmit their beacon, RobotBeacon, every second.
This contains:
* Identification of the robot, to be listed as active.
* The current time at the time of transmission, to detect problems in clock synchronization.

When a robot sets up a private channel, it should stop transmitting on the public channel.
The tablet also transmits its state on the public channel, but robots should ignore it.
Robots should only trust tablet information received in the RoahRsbbBeacon.

#### Private channel

When a robot is active in a benchmark, a private channel is set up for communication.
This channel is encrypted with the team password, to avoid interference from other sources.
This method of protection is only sufficient to avoid honest mistakes in a practical way, not deliberate forgery of messages.

The RSBB transmits benchmark_state containing:
* The code of the specific benchmark to execute.
* The benchmark state.
* An acknowledgment of the last message received from the robot, to avoid eternal retransmission of data.
* Generic goal. Used by the scripted benchmarks to communicate a goal to the robot.

The robot transmits the robot_state containing:
* The current time at the time of transmission, to detect problems in clock synchronization.
* Number of offline data messages saved. Can also be the size of saved data. This is considered to be a fail-safe feature to check whether the robot is recording the mandatory offline data.
* The robot state.
* General goal result. Used in scripted benchmarks to communicate the result of the executed goal.
* Notifications issued by the robot. Used in TBM “Welcoming Visitors” and “Object Manipulation” FBM.
* Activation event. Used in TBM “Welcoming Visitors”.
* Visitor identification. Used in TBM “Welcoming Visitors”.
* Final command identified by the robot. Used in TBM “Catering for Granny Annie’s Comfort”.
* Home devices control commands. Used in TBM “Catering for Granny Annie’s Comfort”.
* Tablet map display command. Used in TBM “Catering for Granny Annie’s Comfort”.

Additional informations may be added to benchmark_state and robot_state when implementing further benchmarks.

## User Interface

`TODO`

## RoAH RSBB Comm

`TODO`
