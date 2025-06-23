import socket
import sys
import json

def main(host='127.0.0.1', port=9000):
    print(f"Connecting to {host}:{port}...")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.connect((host, port))
            print("Connected. Waiting for data...")
            buffer = ''
            while True:
                data = s.recv(4096)
                if not data:
                    print("Connection closed by server.")
                    break
                buffer += data.decode('utf-8')
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip():
                        try:
                            log_entry = json.loads(line)
                            print(json.dumps(log_entry, indent=2))
                        except json.JSONDecodeError:
                            print(f"Malformed JSON: {line}")
        except KeyboardInterrupt:
            print("\nDisconnected.")
        except Exception as e:
            print(f"Error: {e}")

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='TCP client for drone logger')
    parser.add_argument('--host', default='127.0.0.1', help='Server host (default: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=9000, help='Server port (default: 9000)')
    args = parser.parse_args()
    main(args.host, args.port)
