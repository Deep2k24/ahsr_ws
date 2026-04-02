#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from flask import Flask, request, jsonify, render_template_string
import threading

app = Flask(__name__)
ros_node = None

# --- HTML FRONTEND TEMPLATE ---
HTML_PAGE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>Techno Robot Control</title>
    <style>
        body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background-color: #121212; color: white; text-align: center; margin: 0; padding: 20px; touch-action: manipulation; }
        h1 { margin-bottom: 10px; color: #00d2ff; }
        .status { margin-bottom: 30px; font-size: 1.2em; font-weight: bold; }
        
        .dpad { display: grid; grid-template-columns: 80px 80px 80px; grid-template-rows: 80px 80px 80px; gap: 15px; justify-content: center; margin-bottom: 40px; }
        .btn-move { background-color: #333; border: 2px solid #00d2ff; color: #00d2ff; font-size: 24px; border-radius: 15px; cursor: pointer; user-select: none; transition: 0.1s; display: flex; align-items: center; justify-content: center; }
        .btn-move:active { background-color: #00d2ff; color: #121212; transform: scale(0.95); }
        .empty { visibility: hidden; }
        
        .controls { display: flex; flex-direction: column; align-items: center; gap: 20px; }
        .btn-voice { background-color: #28a745; color: white; border: none; padding: 15px 40px; font-size: 20px; border-radius: 30px; font-weight: bold; width: 80%; max-width: 300px; cursor: pointer; }
        .btn-voice.active { background-color: #ffc107; color: #000; }
        
        .btn-estop { background-color: #dc3545; color: white; border: none; padding: 20px 40px; font-size: 24px; border-radius: 10px; font-weight: bold; width: 80%; max-width: 300px; cursor: pointer; box-shadow: 0 4px 10px rgba(220, 53, 69, 0.5); text-transform: uppercase;}
        .btn-estop.active { background-color: #8b0000; box-shadow: inset 0 4px 10px rgba(0,0,0,0.5); border: 2px solid #ff0000; }
    </style>
</head>
<body>

    <h1>Techno Control Panel</h1>
    <div id="connection-status" class="status" style="color: #28a745;">Connected to ROS 2</div>

    <div class="dpad">
        <div class="empty"></div>
        <div class="btn-move" id="btn-up">▲</div>
        <div class="empty"></div>
        <div class="btn-move" id="btn-left">◀</div>
        <div class="btn-move" style="border-color: #555; color: #555;" id="btn-stop">◼</div>
        <div class="btn-move" id="btn-right">▶</div>
        <div class="empty"></div>
        <div class="btn-move" id="btn-down">▼</div>
        <div class="empty"></div>
    </div>

    <div class="controls">
        <button id="btn-voice" class="btn-voice" onclick="toggleVoice()">🎤 Enable Voice Node</button>
        <button id="btn-estop" class="btn-estop" onclick="toggleEStop()">🛑 EMERGENCY STOP</button>
    </div>

    <script>
        let moveInterval;
        let isVoiceActive = false;
        let isEStopActive = false;
        const linearSpeed = 0.3;
        const angularSpeed = 0.5;

        function sendTwist(linear, angular) {
            fetch('/api/cmd_vel', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ linear: linear, angular: angular })
            }).catch(err => console.error("Error sending twist:", err));
        }

        function startMove(linear, angular) {
            if (isEStopActive) return;
            sendTwist(linear, angular);
            moveInterval = setInterval(() => sendTwist(linear, angular), 100); 
        }

        function stopMove() {
            clearInterval(moveInterval);
            sendTwist(0.0, 0.0);
        }

        const setupBtn = (id, lin, ang) => {
            const btn = document.getElementById(id);
            btn.addEventListener('touchstart', (e) => { e.preventDefault(); startMove(lin, ang); });
            btn.addEventListener('touchend', (e) => { e.preventDefault(); stopMove(); });
            btn.addEventListener('mousedown', (e) => { startMove(lin, ang); });
            btn.addEventListener('mouseup', stopMove);
            btn.addEventListener('mouseleave', stopMove);
        };

        setupBtn('btn-up', linearSpeed, 0.0);
        setupBtn('btn-down', -linearSpeed, 0.0);
        setupBtn('btn-left', 0.0, angularSpeed);
        setupBtn('btn-right', 0.0, -angularSpeed);
        setupBtn('btn-stop', 0.0, 0.0);

        function toggleVoice() {
            isVoiceActive = !isVoiceActive;
            const btn = document.getElementById('btn-voice');
            btn.innerText = isVoiceActive ? "🎙️ Voice Node Listening..." : "🎤 Enable Voice Node";
            btn.className = isVoiceActive ? "btn-voice active" : "btn-voice";
            
            fetch('/api/toggle_voice', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ active: isVoiceActive })
            }).then(res => res.json()).then(data => {
                if(!data.success) {
                    alert("Failed to toggle voice: " + data.message);
                    isVoiceActive = !isVoiceActive; 
                }
            });
        }

        function toggleEStop() {
            fetch('/api/estop', { method: 'POST' })
            .then(res => res.json()).then(data => {
                isEStopActive = data.estop_active;
                const btn = document.getElementById('btn-estop');
                if(isEStopActive) {
                    btn.className = "btn-estop active";
                    btn.innerText = "⚠️ E-STOP ACTIVE (CLICK TO RESET)";
                    stopMove(); 
                } else {
                    btn.className = "btn-estop";
                    btn.innerText = "🛑 EMERGENCY STOP";
                }
            });
        }
    </script>
</body>
</html>
"""

class WebBridgeNode(Node):
    def __init__(self):
        super().__init__('web_gui_node')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.voice_client = self.create_client(SetBool, 'toggle_techno_voice')
        self.estop_active = False

    def send_cmd_vel(self, linear: float, angular: float):
        if self.estop_active:
            return 
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    def trigger_estop(self):
        self.estop_active = not self.estop_active
        if self.estop_active:
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
        return self.estop_active

    def toggle_voice(self, active: bool):
        if not self.voice_client.wait_for_service(timeout_sec=2.0):
            return False, "Voice Node Service not running."
        
        req = SetBool.Request()
        req.data = active
        self.voice_client.call_async(req)
        return True, "Success"

@app.route('/')
def index():
    return render_template_string(HTML_PAGE)

@app.route('/api/cmd_vel', methods=['POST'])
def cmd_vel():
    data = request.json
    ros_node.send_cmd_vel(data.get('linear', 0.0), data.get('angular', 0.0))
    return jsonify({"status": "ok"})

@app.route('/api/estop', methods=['POST'])
def estop():
    current_state = ros_node.trigger_estop()
    return jsonify({"estop_active": current_state})

@app.route('/api/toggle_voice', methods=['POST'])
def toggle_voice():
    data = request.json
    success, msg = ros_node.toggle_voice(data.get('active', False))
    return jsonify({"success": success, "message": msg})


def main(args=None):
    global ros_node
    rclpy.init(args=args)
    
    ros_node = WebBridgeNode()
    
    flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False))
    flask_thread.daemon = True
    flask_thread.start()

    ros_node.get_logger().info("🌐 Web GUI Server running! Access it via http://<YOUR_ROBOT_IP>:5000")

    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
