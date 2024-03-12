#!/usr/bin/env python

from labelmaker_encode import encode_raster_transfer, read_png
import argparse
import sys
import signal
import ptcbp
import ptstatus
import threading
import time
from PyQt6 import QtCore
from PyQt6 import QtBluetooth

class BluetoothSocketWorker(QtCore.QObject):
    bytesReceived = QtCore.pyqtSignal(bytearray)
    connected = QtCore.pyqtSignal()

    def __init__(self, addr):
        super().__init__()
        self.dev = QtBluetooth.QBluetoothLocalDevice(self)
        self.dev.deviceConnected.connect(self.deviceConnected)
        self.dev.errorOccurred.connect(self.socketError)
        self.dev.powerOn()
        self.recvbuf = bytearray()
 
        self.addr = QtBluetooth.QBluetoothAddress(addr)
        print(self.dev.pairingStatus(self.addr))
        print("valid:")
        print(self.dev.isValid())

        self.open()


    def open(self):
        self.sock = QtBluetooth.QBluetoothSocket(QtBluetooth.QBluetoothServiceInfo.Protocol.RfcommProtocol, self)
        self.sock.connected.connect(self.socketConnected)
        self.sock.readyRead.connect(self.socketReadyRead)
        self.sock.disconnected.connect(self.socketDisconnected)
        self.sock.errorOccurred.connect(self.socketError)
        self.sock.stateChanged.connect(self.socketStateChanged)
        self.sock.connectToService(self.addr, 1)

    def socketDisconnected(self):
        print("socket disconneted")

    def deviceConnected(self, service):
        if (self.addr == service):
            print("Drukarak")
            # self.pairing = True
            # self.dev.requestPairing(self.addr, QtBluetooth.QBluetoothLocalDevice.AuthorizedPaired)

    @QtCore.pyqtSlot(bytes)
    def write(self, data):
        toWrite = len(data)

        while (toWrite > 0):
            writtenCount = self.sock.write(data[len(data)-toWrite:])
            toWrite -= writtenCount
            print("Written count", writtenCount)
            if (len(data) is not writtenCount):
                print("missing written bytes")

    @QtCore.pyqtSlot(int)
    def read(self, readLen):
        self.sock.read(readLen)
        self.sock.waitForReadyRead(5000)  # Read timeout of 5 seconds

    def socketStateChanged(self, state):
        print(state)

    def socketError(self,error):
        print("Socket error")
        print(error)

    def socketConnected(self):
        print("Bluetooth socket worker emit")
        self.connected.emit()


    def disconnectedFromBluetooth(self):
        print('Disconnected from bluetooth')

    def socketReadyRead(self):
        print("Received data incoming")
        incomingData = bytearray()
        incomingData += self.sock.readAll().data()

        self.bytesReceived.emit(incomingData)

class BluetoothSocket(QtCore.QObject):
    readRequested = QtCore.pyqtSignal(int)
    writeRequested = QtCore.pyqtSignal(bytes)

    def __init__(self, addr):
        super().__init__()
        self.btThread = QtCore.QThread()
        self.btThread.start()
        self.socketWorker = BluetoothSocketWorker(addr)
        self.socketWorker.moveToThread(self.btThread)

        self.dataReceivedSem = QtCore.QWaitCondition()
        self.socketOpenedSem = threading.Semaphore(0)
        self.mutex = QtCore.QMutex()

        # holds incoming data read in readyRead
        self.recvBuffer = bytearray()
        self.recvBufferMutex = threading.Lock()
        self.socketWorker.connected.connect(self.socketConnected, type=QtCore.Qt.ConnectionType.QueuedConnection)
        self.socketWorker.bytesReceived.connect(self.socketReadyRead, type=QtCore.Qt.ConnectionType.DirectConnection)
        self.writeRequested.connect(self.socketWorker.write, type=QtCore.Qt.ConnectionType.DirectConnection)
        self.readRequested.connect(self.socketWorker.read)

    def write(self, data):
        self.writeRequested.emit(data)

    def read(self, readLen, timeout=5):
        # with self.recvBufferMutex:
        #     self.recvBuffer = bytearray()
        print("reading", readLen)
        toRead = readLen
        receivedBytes = bytearray()
        self.readRequested.emit(readLen)

        start_time = time.time()

        while toRead > 0:
            elapsed_time = time.time() - start_time
            if timeout is not None and elapsed_time >= timeout:
                raise TimeoutError("Read timeout")

            print(toRead)
            with self.recvBufferMutex:
                if len(self.recvBuffer) > 0:
                    print(len(self.recvBuffer))
                    requestedBytes = min(len(self.recvBuffer), toRead)
                    receivedBytes += self.recvBuffer[0:requestedBytes]
                    self.recvBuffer = self.recvBuffer[requestedBytes:]
                    toRead -= requestedBytes

            if toRead > 0:
                self.mutex.lock()
                self.dataReceivedSem.wait(self.mutex, timeout * 1000)
                self.mutex.unlock()
        print(f"Successfully read {readLen} bytes")
        print(receivedBytes)
        return bytes(receivedBytes)

    @QtCore.pyqtSlot()
    def socketConnected(self):
        print("dupa")
        self.onOpenCallback()
        # self.sock.write('A'.encode())
        pass

    def onOpen(self, onOpenCallback):
        self.onOpenCallback = onOpenCallback

    @QtCore.pyqtSlot(bytes)
    def socketReadyRead(self, incomingData):
        print("received data", incomingData)
        with self.recvBufferMutex:
            self.recvBuffer += incomingData
        self.dataReceivedSem.wakeAll()

    def disconnect(self):
        self.socketWorker.disconnectedFromBluetooth()
        self.btThread.quit()

BARS = '_▁▂▃▄▅▆▇█'

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('bdaddr', help='BDADDR of the printer.')
    p.add_argument('-i', '--image', help='Image file to print.')
    p.add_argument('-n', '--no-print', help='Only configure the printer and send the image but do not send print command.', action='store_true')
    p.add_argument('-F', '--no-feed', help='Disable feeding at the end of the print (chaining).')
    p.add_argument('-a', '--auto-cut', help='Enable auto-cutting (or print label boundary on e.g. PT-P300BT).')
    p.add_argument('-m', '--end-margin', help='End margin (in dots).', default=0, type=int)
    p.add_argument('-r', '--raw', help='Send the image to printer as-is without any pre-processing.', action='store_true')
    p.add_argument('-C', '--nocomp', help='Disable compression.', action='store_true')
    return p, p.parse_args()

def reset_printer(ser):
    print("Resetting printer")
    # Flush print buffer
    ser.write(b"\x00" * 64)

    # Initialize
    ser.write(ptcbp.serialize_control('reset'))

    # Enter raster graphics (PTCBP) mode
    ser.write(ptcbp.serialize_control('use_command_set', ptcbp.CommandSet.ptcbp))

def configure_printer(ser, raster_lines, tape_dim, compress=True, chaining=False, auto_cut=False, end_margin=0):
    reset_printer(ser)

    type_, width, length = tape_dim
    # Set media & quality
    ser.write(ptcbp.serialize_control_obj('set_print_parameters', ptcbp.PrintParameters(
        active_fields=(ptcbp.PrintParameterField.width |
                       ptcbp.PrintParameterField.quality |
                       ptcbp.PrintParameterField.recovery),
        media_type=type_,
        width_mm=width, # Tape width in mm
        length_mm=length, # Label height in mm (0 for continuous roll)
        length_px=raster_lines, # Number of raster lines in image data
        is_follow_up=0, # Unused
        sbz=0, # Unused
    )))

    pm, pm2 = 0, 0
    if not chaining:
        pm2 |= ptcbp.PageModeAdvanced.no_page_chaining
    if auto_cut:
        pm |= ptcbp.PageMode.auto_cut

    # Set print chaining off (0x8) or on (0x0)
    ser.write(ptcbp.serialize_control('set_page_mode_advanced', pm2))

    # Set no mirror, no auto tape cut
    ser.write(ptcbp.serialize_control('set_page_mode', pm))

    # Set margin amount (feed amount)
    ser.write(ptcbp.serialize_control('set_page_margin', end_margin))

    # Set compression mode: TIFF
    ser.write(ptcbp.serialize_control('compression', ptcbp.CompressionType.rle if compress else ptcbp.CompressionType.none))

def do_print_job(ser, args, data):
    print('=> Querying printer status...')

    reset_printer(ser)

    # Dump status
    ser.write(ptcbp.serialize_control('get_status'))
    status = ptstatus.unpack_status(ser.read(32))
    ptstatus.print_status(status)

    if status.err != 0x0000 or status.phase_type != 0x00 or status.phase != 0x0000:
        print('** Printer indicates that it is not ready. Refusing to continue.')
        sys.exit(1)

    print('=> Configuring printer...')

    raster_lines = len(data) // 16
    configure_printer(ser, raster_lines, (status.tape_type,
                                          status.tape_width,
                                          status.tape_length),
                      chaining=args.no_feed,
                      auto_cut=args.auto_cut,
                      end_margin=args.end_margin,
                      compress=not args.nocomp)

    # Send image data
    print(f"=> Sending image data ({raster_lines} lines)...")
    sys.stdout.write('[')
    fullImage = bytearray()
    for line in encode_raster_transfer(data, args.nocomp):
        if line[0:1] == b'G':
            sys.stdout.write(BARS[min((len(line) - 3) // 2, 7) + 1])
        elif line[0:1] == b'Z':
            sys.stdout.write(BARS[0])
        sys.stdout.flush()
        fullImage += line
    sys.stdout.write(']')
    ser.write(bytes(fullImage))

    print()
    print("=> Image data was sent successfully. Printing will begin soon.")

    if not args.no_print:
        print("Gonna write serialize control")
        # Print and feed
        ser.write(ptcbp.serialize_control('print'))
        print("Print command sent")

        # Dump status that the printer returns
        status = ptstatus.unpack_status(ser.read(32))
        ptstatus.print_status(status)

    print("=> All done.")

btsocket = None

def main(app):
    p, args = parse_args()

    data = None
    if args.image is None:
        p.error('An image must be specified for printing job.')
    else:
        # Read input image into memory
        if args.raw:
            data = read_png(args.image, False, False, False)
        else:
            data = read_png(args.image)


    # ser = serial.Serial(
    #     port='/dev/tty.PT-P300BT1970',
    #     baudrate=19200
    # )

    baddr = args.bdaddr
    btsocket = BluetoothSocket(baddr)

    thread = QtCore.QThread()
    thread.start()
    btsocket.moveToThread(thread)

    # ctrl +c handler
    def signal_handler(sig, frame):
        print('You pressed Ctrl+C!')
        reset_printer(btsocket)
        btsocket.disconnect()
        btsocket.btThread.quit()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)


    def print_job():
        print("elo")
        try:
            assert data is not None
            do_print_job(btsocket, args, data)
        except Exception as e:
            print("Exception", e)
            reset_printer(btsocket)
            btsocket.disconnect()
            btsocket.btThread.quit()
            sys.exit(0)
        finally:
            reset_printer(btsocket)
            print("Finally")
            btsocket.disconnect()
            btsocket.btThread.quit()
            sys.exit(0)

    btsocket.onOpen(lambda: print_job())
    sys.exit(app.exec())

if __name__ == '__main__':
    import sys, os
    os.environ['QT_EVENT_DISPATCHER_CORE_FOUNDATION'] = '1'
    app = QtCore.QCoreApplication(sys.argv)
    main(app)
