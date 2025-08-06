from flask import Flask, render_template
from flask_socketio import SocketIO
from can_reader import start_can_reader, get_latest_data

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app)

@app.route("/")
def index():
    return render_template("dashboard.html")

# Emit data every second
@socketio.on('connect')
def handle_connect():
    print("Client connected")

    def stream_data():
        while True:
            socketio.sleep(1)
            socketio.emit("update", get_latest_data())

    socketio.start_background_task(target=stream_data)

if __name__ == "__main__":
    start_can_reader()  # Start CAN reading in background
    socketio.run(app, host="0.0.0.0", port=5001)
