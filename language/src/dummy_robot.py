import socket
import threading
import time

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

def start_client(host='localhost', port=50050, send=True):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))
    s.sendall(b'BOEBOT')

    receive_thread = threading.Thread(target=receive_messages, args=(s,))

    if send:
        send_thread = threading.Thread(target=send_messages, args=(s,))

    receive_thread.start()

    if send:
        send_thread.start()

    receive_thread.join()

    if send:
        send_thread.join()

def try_start_headless():
    """Turn off the send thread, so that it doesn't 
    ask for any input from the command line. This is mainly for
    running the headless version, which should just recieve
    packets from the server and do nothing with them.
    """
    while True:
        try:
            start_client(send=False)
        except Exception as e:
            print(e)
            time.sleep(2)

if __name__ == "__main__":
    start_client()
