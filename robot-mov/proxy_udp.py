from flask import Flask, request
import socket

app = Flask(__name__)

ESP32_IP = "10.254.27.75"        # CAMBIAR POR LA DIRECCIÓN IP QUE SE ASIGNE AL ESP
ESP32_PORT = 3333

@app.route('/send', methods=['GET'])
def send_udp():
    mode = request.args.get("mode")
    args = request.args.getlist("arg")

    if not mode or not args:
        return "Missing parameters", 400

    try:
        message = f"{mode} {' '.join(args)}"
        print(f"[PROXY] Sending: {message} to {ESP32_IP}:{ESP32_PORT}")

        # Crear socket UDP
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(message.encode(), (ESP32_IP, ESP32_PORT))
        sock.close()

        return f"Sent: {message}", 200

    except Exception as e:
        print(f"[ERROR] {e}")
        return f"Error: {str(e)}", 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)