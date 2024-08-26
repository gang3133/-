import serial



def recieve_data(bluetoothSerial):
    buffer=""
    maxLength = 10
    while True:
        oneByte = bluetoothSerial.read(1)
        if oneByte == b"\n":    
            break
        else:
            buffer += oneByte.decode()
            if len(buffer) >= maxLength:  
                print("데이터 길이 초과")
                break  

    print (buffer.strip())
    return buffer.strip()

def send_data(bluetoothSerial ,data):
    bluetoothSerial.write(bytearray(data))