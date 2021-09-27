const rclnodejs = require('rclnodejs');
const { QoS } = rclnodejs;

const WebSocket = require('ws');
const wss = new WebSocket.Server({ port: 8080 });

rclnodejs.init().then(() => {
    const node = rclnodejs.createNode('subscription_example_node');
    let qos = new QoS();
    qos.reliability = QoS.ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

    var connected = []
    node.createSubscription('std_msgs/msg/Float32', '/weather_station/humidity', { qos, }, msg => {
      console.log(`Received humidity: ${typeof msg}`, msg);
      connected.forEach(e => {
        e.send(JSON.stringify({"humidity":msg.data}))
      })
    });

    node.createSubscription('std_msgs/msg/Float32', '/weather_station/temperature', { qos, }, msg => {
      console.log(`Received temperature: ${typeof msg}`, msg);
      connected.forEach(e => {
        e.send(JSON.stringify({"temperature":msg.data}))
      })
    });

    wss.on('connection', function connection(ws) {
      // console.log("WS connected")
      connected.push(ws)
    });

    rclnodejs.spin(node);
  });