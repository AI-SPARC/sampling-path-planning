import serial
import argparse
import threading
import time

def read_serial():
    while True:
        try:
            if ser.in_waiting > 0:
                # Pula o que for lixo ilegível.
                data = ser.readline().decode('utf-8', errors='ignore')
                if data:
                    print(f"\nEscutando: {data}", end='')
                    print("> ", end='', flush=True)
        except Exception as e:
            print(f"\nErro de leitura: {e}")
            break

def main():
    global ser
    parser = argparse.ArgumentParser(description='Controlador RoArm-M2')
    parser.add_argument('port', type=str, help='Porta COM (ex: COM4)')
    
    args = parser.parse_args()

    print(f"Conectando em {args.port} a 115200...")
    
    ser = serial.Serial(args.port, baudrate=115200, timeout=1)

    # Inicia a thread que fica ouvindo o robô
    serial_recv_thread = threading.Thread(target=read_serial)
    serial_recv_thread.daemon = True
    serial_recv_thread.start()

    print("Conectado!")
    print("> ", end='', flush=True)

    try:
        while True:
            command = input()
            if command.strip(): # Só envia se não estiver vazio
                # Adiciona \r\n para garantir que o robô entenda o fim do comando
                ser.write((command + '\r\n').encode())
    except KeyboardInterrupt:
        print("\nSaindo...")
    finally:
        ser.close()

if __name__ == "__main__":
    main()