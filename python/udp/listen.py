#!/usr/bin/env python3
import socket
from colorama import Fore
import time

def main():
    start_time = time.monotonic()
    pkt_cnt = 0
    addr = ("0.0.0.0", 12321)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(addr)

    print(f"Listening on: {addr}")

    try:
        while True:
            data, addr = sock.recvfrom(64)
            if data:
                pkt_cnt += 1
                # print(f"recv'ed: {data} from {addr}")
                print(f"Address: {Fore.GREEN}{addr}{Fore.RESET}")
                print(f"Data: {Fore.CYAN}{data}{Fore.RESET}")
    except KeyboardInterrupt:
        print("ctrl-c ...")
    finally:
        end_time = time.monotonic()
        len_time = end_time - start_time
        print(f"Packets: {pkt_cnt}")
        print(f"Time: {len_time:.3f} sec")
        print(f"Packet Rate: {pkt_cnt / len_time:.1f} Hz")
        sock.close()

main()