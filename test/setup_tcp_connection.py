import socket
import threading

def receive_messages(sock):
    while True:
        try:
            data = sock.recv(1024)
            if not data:
                print("Server closed the connection.")
                sock.close()
                break
            print(f"Received: {data.decode()}")
        except OSError:
            print("Socket closed.")
            break

def send_messages(sock):
    while True:
        try:
            message = input("Enter message to send: ")
            if message.lower() == 'exit':
                sock.close()
                break
            sock.sendall(message.encode())
        except OSError:
            print("Socket closed.")
            break

def start_client(host='localhost', port=50050):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))
    s.sendall(b'BOEBOT')

    receive_thread = threading.Thread(target=receive_messages, args=(s,))
    send_thread = threading.Thread(target=send_messages, args=(s,))

    receive_thread.start()
    send_thread.start()

    receive_thread.join()
    send_thread.join()

if __name__ == "__main__":
    start_client()
