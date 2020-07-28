const rclnodejs = require('rclnodejs');
const WebSocket = require('ws');
const wss = new WebSocket.Server({ port: 8080 });

rclnodejs.init().then(() => {
    const node = rclnodejs.createNode('subscription_example_node');

    var connected = []
    node.createSubscription('std_msgs/msg/Int32', '/weather_station/temperature', msg => {
      // console.log(`Received message: ${typeof msg}`, msg);
      connected.forEach(e => {
        e.send(JSON.stringify({"temperature":msg.data}))
      })
    });

    node.createSubscription('std_msgs/msg/Int32', '/weather_station/humidity', msg => {
      // console.log(`Received message: ${typeof msg}`, msg);
      connected.forEach(e => {
        e.send(JSON.stringify({"humidity":msg.data}))
      })
    });
  
    wss.on('connection', function connection(ws) {
      // console.log("WS connected")
      connected.push(ws)
    });
  
    rclnodejs.spin(node);
  });