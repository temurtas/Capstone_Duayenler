
import socket
import sys


def client_program():
    host = "144.122.86.47" # server's ip
    port = 5000  # socket server port number

    client_socket = socket.socket()  # instantiate
    client_socket.connect((host, port))  # connect to the server

    message = "ID00"  # catching message

    client_socket.send(message.encode())  # send message
    data = client_socket.recv(1024).decode()  # receive response
    print('Received from server: ' + data)  # show in terminal

    if data == "ID11":
        # if it is not acknowledged return
        client_socket.close()  # close the connection
        return

    message = "ID10"  # stop message
    client_socket.send(message.encode())  # send message
    client_socket.close()  # close the connection
    sys.exit()

def main():
    while(True):
        sensor = input("Is it 5 cm?")
        if sensor == "0":
            print("Don't send catching message")
        elif sensor == "1":
            client_program()
        else:
            print("Not a valid input!!")

main()
