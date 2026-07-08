// Server ports — loaded from /api/server_config (ROS2 defaults: web 8080, pc2 ws 8081).
window.ROS_SLAM_WEBUI = {
    webPort: 8080,
    pc2WsPort: 8081,
    rosbridgePort: 9090,
    ready: false,
};

function getWebSocketHost() {
    return window.location.hostname || 'localhost';
}

function getPc2WsPort() {
    return window.ROS_SLAM_WEBUI.pc2WsPort || 8081;
}

function getPc2WsUrl(host) {
    const h = host || getWebSocketHost();
    return `ws://${h}:${getPc2WsPort()}`;
}

function getRosbridgePort() {
    const configuredPort = Number(window.ROS_SLAM_WEBUI && window.ROS_SLAM_WEBUI.rosbridgePort);
    if (Number.isFinite(configuredPort) && configuredPort > 0) {
        return configuredPort;
    }
    return 9090;
}

function getRosbridgeUrl(host) {
    const h = host || getWebSocketHost();
    return `ws://${h}:${getRosbridgePort()}`;
}

function getRosNotConnectedHint() {
    const rosVersion = (typeof window._rosVersion === 'number') ? window._rosVersion : 2;
    if (rosVersion === 1) {
        return 'Not connected to ROS. Make sure rosbridge_server is running:\n\n'
            + 'roslaunch ros_slam_webui ros_slam_webui.launch\n'
            + '# or\n'
            + 'roslaunch rosbridge_server rosbridge_websocket.launch port:=9090';
    }
    return 'Not connected to ROS. Make sure rosbridge_server is running:\n\n'
        + 'ros2 launch rosbridge_server rosbridge_websocket_launch.xml';
}

window.ROS_SLAM_WEBUI_READY = fetch('/api/server_config')
    .then((r) => r.json())
    .then((cfg) => {
        if (cfg.web_port != null) window.ROS_SLAM_WEBUI.webPort = cfg.web_port;
        if (cfg.pc2_ws_port != null) window.ROS_SLAM_WEBUI.pc2WsPort = cfg.pc2_ws_port;
        if (cfg.rosbridge_port != null) window.ROS_SLAM_WEBUI.rosbridgePort = cfg.rosbridge_port;
        window.ROS_SLAM_WEBUI.ready = true;
        document.dispatchEvent(new CustomEvent('ros-slam-webui-ports-ready'));
        return window.ROS_SLAM_WEBUI;
    })
    .catch(() => {
        window.ROS_SLAM_WEBUI.ready = true;
        document.dispatchEvent(new CustomEvent('ros-slam-webui-ports-ready'));
        return window.ROS_SLAM_WEBUI;
    });

function ensureWebuiPortsReady() {
    if (window.ROS_SLAM_WEBUI.ready) return Promise.resolve(window.ROS_SLAM_WEBUI);
    return window.ROS_SLAM_WEBUI_READY;
}
