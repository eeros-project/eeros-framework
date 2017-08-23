* ROS has to be installed on the host computer, otherwise the ROS-example will not be compiled
* You need the wrapper library 'libroseeros.so'. If you don't have installed this library, copy it to this folder.
* Start the roscore with 'roscore'
* Before you run the example, start 'rosNodeTalker'. This will create a ROS node which publishes messages to multiple test topics
* Run the example in a different terminal with 'sudo -E ./rosExample -c HalConfigRos.json'
* You can monitor all started nodes and topics with the 'rqt node graph' (run rqt -> Plugins -> Introspection -> Node Graph)
* To examine the values of the published messages, you can use 'rqt topic monitor' (run rqt -> Plugins -> Topics -> Topic Monitor)