import socket
import csv

udpPort = 1234
bufferSize = 20

udpsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udpsock.bind(('0.0.0.0', udpPort))

with open('raw.csv', mode='w') as csvFile:
    csvWriter = csv.writer(csvFile, delimiter=',')
    csvWriter.writerow(['time', 'angle'])
    positive = True
    while True:
        msg = udpsock.recvfrom(bufferSize)
        msgStr = msg[0].decode('ascii')
        if len(msgStr) < 15:
            timeStamp, angle = msgStr.split('/', 2)
            if not positive:
                angle = str(-1 * int(angle))
            positive = not positive
            csvWriter.writerow([timeStamp, angle])
            print('time: ', timeStamp)
            print('angle: ', angle)
