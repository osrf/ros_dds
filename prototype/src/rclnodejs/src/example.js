var rclnodejs = require('/Users/william/devel/ros_dds/prototype/devel/lib/rclnodejs.node')

rclnodejs.init();

var node = rclnodejs.create_node('listener');

var sub = rclnodejs.create_subscription('chatter', 10, function(msg) {
    console.log(msg.data);
});
