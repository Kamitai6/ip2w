import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 5000))

print("Listening on UDP 5000...")
while True:
    data, addr = sock.recvfrom(1024)
    print(f"[{addr[0]}]: {data.decode('utf-8', errors='ignore')}", end='')