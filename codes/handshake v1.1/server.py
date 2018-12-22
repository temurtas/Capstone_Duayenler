import socket
import sys


def server_program():
    # get the hostname
    host = socket.gethostname()
    port = 5000  # initiate port no above 1024

    server_socket = socket.socket()  # get instance
    # look closely. The bind() function takes tuple as argument
    server_socket.bind((host, port))  # bind host address and port together

    # configure how many client the server can listen simultaneously
    server_socket.listen(2)
    (conn, address) = server_socket.accept()  # accept new connection
    print("Connection from: " + str(address))

    # receive data stream. it won't accept data packet greater than 1024 bytes
    data = conn.recv(1024).decode()
    print("from connected user: " + str(data))
    ack = input("Should I acknowledge?")
    if ack == "0":
        data = "ID11(REJ)"   #not acknowledged message
        conn.send(data.encode())  # send data to the client
        conn.close()  # close the connection
        return
    data = "ID01(ACK)"
    conn.send(data.encode())  # send data to the client

    data = conn.recv(1024).decode() # receive data stream
    print("from connected user: " + str(data))

    conn.close()  # close the connection
    input("Press enter to close terminal.")
    sys.exit()

def main():
    while (True):
        server_program()

main()