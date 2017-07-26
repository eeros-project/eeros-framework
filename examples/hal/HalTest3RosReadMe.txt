* ROS has to be installed on the host computer, otherwise the ROS-example will not be compiled
* You need the wrapper library 'libroseeros.so'. If you don't have installed this library, copy it to this folder.
* Before you run the example, start 'rosNodeTalker'. This will create a ROS node and multiple topics and post messages to the topics
* Run the example in a different terminal with 'sudo -E ./halTest3Ros -c HalTest3ConfigRos.json'
* You can monitor all started nodes and topics with the 'rqt node graph' (run rqt -> Plugins -> Introspection -> Node Graph)
* To examine the values of the published messages, you can use 'rqt topic monitor'