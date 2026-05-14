#!/usr/bin/env python3
import argparse
import struct
import sys
import time
from pathlib import Path

import serial

SOF = 0xA5

CMD_START = 0x01
CMD_DATA = 0x02
CMD_END_INPUT = 0x03
CMD_RUN = 0x04

CMD_ACK = 0x79
CMD_NACK = 0x1F
CMD_COMPRESSED_DATA = 0x81
CMD_END_OUTPUT = 0x82
CMD_ERROR = 0xE0

CONTAINER_MAGIC = b"ACS1"
MODE_RAW = 0
MODE_ARITH = 1

ERR_MAP = {
    0x01: "ERR_BAD_CHECKSUM",
    0x02: "ERR_BAD_STATE",
    0x03: "ERR_OVERSIZE",
    0x04: "ERR_TIMEOUT",
    0x05: "ERR_ENCODER",
}


def checksum(cmd: int, payload: bytes) -> int:
    length = len(payload)
    value = cmd ^ (length & 0xFF) ^ ((length >> 8) & 0xFF)
    for byte in payload:
        value ^= byte
    return value & 0xFF


def pack_frame(cmd: int, payload: bytes = b"") -> bytes:
    length = len(payload)
    return bytes([SOF, cmd, length & 0xFF, (length >> 8) & 0xFF]) + payload + bytes([checksum(cmd, payload)])


def read_exact(ser: serial.Serial, count: int) -> bytes:
    data = bytearray()
    while len(data) < count:
        chunk = ser.read(count - len(data))
        if not chunk:
            raise TimeoutError(f"Timeout lendo {count} bytes ({len(data)} recebidos)")
        data.extend(chunk)
    return bytes(data)


def read_frame(ser: serial.Serial) -> tuple[int, bytes]:
    while True:
        sof = ser.read(1)
        if not sof:
            raise TimeoutError("Timeout aguardando SOF")
        if sof[0] == SOF:
            break

    header = read_exact(ser, 3)
    cmd = header[0]
    length = header[1] | (header[2] << 8)
    payload = read_exact(ser, length)
    rx_checksum = read_exact(ser, 1)[0]
    expected = checksum(cmd, payload)
    if rx_checksum != expected:
        raise ValueError(f"Checksum invalido: rx=0x{rx_checksum:02X} expected=0x{expected:02X}")
    return cmd, payload


def wait_ack(ser: serial.Serial, expected_cmd: int, verbose: bool, early_compressed: bytearray) -> None:
    while True:
        cmd, payload = read_frame(ser)

        if cmd == CMD_ACK:
            echoed = payload[0] if payload else None
            if echoed != expected_cmd:
                raise RuntimeError(f"ACK inesperado: 0x{echoed:02X} (esperado 0x{expected_cmd:02X})")
            if verbose:
                print(f"[ACK] comando=0x{echoed:02X}")
            return

        if cmd == CMD_COMPRESSED_DATA:
            early_compressed.extend(payload)
            if verbose:
                print(f"[RX] COMPRESSED_DATA precoce {len(payload)} bytes (acumulado={len(early_compressed)})")
            continue

        if cmd == CMD_NACK:
            code = payload[0] if payload else 0xFF
            raise RuntimeError(f"NACK: {ERR_MAP.get(code, f'0x{code:02X}')}")

        if cmd == CMD_ERROR:
            code = payload[0] if payload else 0xFF
            raise RuntimeError(f"ERROR: {ERR_MAP.get(code, f'0x{code:02X}')}")

        if cmd == CMD_END_OUTPUT:
            raise RuntimeError("END_OUTPUT recebido antes do ACK esperado")

        raise RuntimeError(f"Resposta inesperada: cmd=0x{cmd:02X}")


def receive_compressed(ser: serial.Serial, verbose: bool, initial_data: bytes = b"") -> bytes:
    compressed = bytearray(initial_data)
    reported_total = None
    if verbose and compressed:
        print(f"[INFO] Ja existem {len(compressed)} bytes comprimidos recebidos durante DATA")
    while True:
        cmd, payload = read_frame(ser)
        if cmd == CMD_COMPRESSED_DATA:
            compressed.extend(payload)
            if verbose:
                print(f"[RX] COMPRESSED_DATA {len(payload)} bytes (acumulado={len(compressed)})")
        elif cmd == CMD_END_OUTPUT:
            if len(payload) != 4:
                raise RuntimeError("END_OUTPUT com payload invalido")
            reported_total = struct.unpack("<I", payload)[0]
            break
        elif cmd == CMD_NACK:
            code = payload[0] if payload else 0xFF
            raise RuntimeError(f"NACK durante recebimento: {ERR_MAP.get(code, f'0x{code:02X}')}")
        elif cmd == CMD_ERROR:
            code = payload[0] if payload else 0xFF
            raise RuntimeError(f"ERROR da placa: {ERR_MAP.get(code, f'0x{code:02X}')}")
        else:
            raise RuntimeError(f"Comando inesperado durante output: 0x{cmd:02X}")

    if reported_total is not None and reported_total != len(compressed):
        raise RuntimeError(f"Tamanho final divergente: placa={reported_total}, host={len(compressed)}")

    return bytes(compressed)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Envia arquivo para STM32 via UART e recebe comprimido.")
    parser.add_argument("--port", required=True, help="Porta serial (ex: /dev/tty.usbserial-0001)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--in", dest="input_path", required=True, help="Arquivo de entrada")
    parser.add_argument("--out", dest="output_path", required=True, help="Arquivo binario de saida")
    parser.add_argument("--max-bytes", type=int, default=4096, help="Limite maximo de entrada")
    parser.add_argument("--chunk", type=int, default=32, help="Payload por pacote DATA (<=32)")
    parser.add_argument("--timeout", type=float, default=1.0, help="Timeout serial em segundos")
    parser.add_argument("--verbose", action="store_true", help="Mostra logs detalhados")
    parser.add_argument(
        "--force-compressed",
        action="store_true",
        help="Salva sempre payload comprimido, mesmo que fique maior que o original",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    input_path = Path(args.input_path)
    output_path = Path(args.output_path)

    if args.chunk <= 0 or args.chunk > 32:
        print("Erro: --chunk deve estar entre 1 e 32", file=sys.stderr)
        return 2

    data = input_path.read_bytes()
    if len(data) > args.max_bytes:
        print(
            f"Erro: arquivo possui {len(data)} bytes e excede limite {args.max_bytes}",
            file=sys.stderr,
        )
        return 2

    print(f"[INFO] Entrada: {input_path} ({len(data)} bytes)")
    print(f"[INFO] Saida: {output_path}")
    print(f"[INFO] Porta: {args.port} @ {args.baud}")

    start_time = time.time()
    with serial.Serial(args.port, args.baud, timeout=args.timeout) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        ser.write(pack_frame(CMD_START, struct.pack("<I", len(data))))
        early_compressed = bytearray()

        wait_ack(ser, CMD_START, args.verbose, early_compressed)

        offset = 0
        packet_count = 0
        while offset < len(data):
            chunk = data[offset : offset + args.chunk]
            ser.write(pack_frame(CMD_DATA, chunk))
            wait_ack(ser, CMD_DATA, args.verbose, early_compressed)
            offset += len(chunk)
            packet_count += 1

        if args.verbose:
            print(f"[INFO] Pacotes DATA enviados: {packet_count}")

        ser.write(pack_frame(CMD_END_INPUT))
        compressed = receive_compressed(ser, args.verbose, bytes(early_compressed))

    elapsed = time.time() - start_time
    if (not args.force_compressed) and (len(compressed) >= len(data)):
        mode = MODE_RAW
        payload = data
        if args.verbose:
            print("[INFO] Sem ganho de compressao: salvando em modo RAW.")
    else:
        mode = MODE_ARITH
        payload = compressed
        if args.verbose:
            print("[INFO] Salvando em modo ARITH.")

    container = bytearray()
    container.extend(CONTAINER_MAGIC)
    container.append(mode)
    container.extend(struct.pack("<I", len(data)))
    container.extend(struct.pack("<I", len(payload)))
    container.extend(payload)
    output_path.write_bytes(container)

    ratio = (len(payload) / len(data)) if data else 0.0
    print(f"[OK] Payload recebido da placa: {len(compressed)} bytes")
    print(f"[OK] Modo salvo no arquivo: {'ARITH' if mode == MODE_ARITH else 'RAW'}")
    print(f"[OK] Tamanho final payload/original: {ratio:.4f}")
    print(f"[OK] Tempo total: {elapsed:.3f} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
