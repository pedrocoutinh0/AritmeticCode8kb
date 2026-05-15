#!/usr/bin/env python3
import argparse
import struct
import sys
from pathlib import Path

CONTAINER_MAGIC = b"ACS1"
MODE_RAW = 0
MODE_ARITH = 1
SYMBOL_COUNT = 256
WNC_TOTAL_MAX = (1 << 14) - 1


class BitReader:
    def __init__(self, data: bytes) -> None:
        self.data = data
        self.pos = 0
        self.cur = 0
        self.nbits = 0

    def read_bit(self) -> int:
        if self.nbits == 0:
            if self.pos >= len(self.data):
                return 0
            self.cur = self.data[self.pos]
            self.pos += 1
            self.nbits = 8
        bit = (self.cur >> 7) & 1
        self.cur = (self.cur << 1) & 0xFF
        self.nbits -= 1
        return bit


def rescale(freq: list[int]) -> int:
    total = 0
    for i, value in enumerate(freq):
        v = (value + 1) >> 1
        if v == 0:
            v = 1
        freq[i] = v
        total += v
    return total


def decode_adaptive(payload: bytes, original_len: int) -> bytes:
    if original_len == 0:
        return b""

    max_value = (1 << 16) - 1
    half = 1 << 15
    first_qtr = 1 << 14
    third_qtr = first_qtr * 3

    freq = [1] * SYMBOL_COUNT
    total = SYMBOL_COUNT
    low = 0
    high = max_value
    br = BitReader(payload)
    value = 0
    for _ in range(16):
        value = ((value << 1) | br.read_bit()) & 0xFFFFFFFF

    out = bytearray()
    for _ in range(original_len):
        range_ = (high - low) + 1
        cum_x = (((value - low) + 1) * total - 1) // range_

        cum = 0
        symbol = 0
        for i, f in enumerate(freq):
            nxt = cum + f
            if nxt > cum_x:
                symbol = i
                break
            cum = nxt
        cum_low = cum
        cum_high = cum + freq[symbol]

        high = low + (range_ * cum_high // total) - 1
        low = low + (range_ * cum_low // total)

        while True:
            if high < half:
                pass
            elif low >= half:
                value -= half
                low -= half
                high -= half
            elif low >= first_qtr and high < third_qtr:
                value -= first_qtr
                low -= first_qtr
                high -= first_qtr
            else:
                break
            low = (low << 1) & max_value
            high = ((high << 1) | 1) & max_value
            value = ((value << 1) | br.read_bit()) & 0xFFFFFFFF

        out.append(symbol)

        freq[symbol] += 1
        total += 1
        if total >= WNC_TOTAL_MAX:
            total = rescale(freq)

    return bytes(out)


def parse_container(blob: bytes) -> tuple[int, int, bytes]:
    if len(blob) < 13:
        raise ValueError("Arquivo muito curto para container")
    if blob[0:4] != CONTAINER_MAGIC:
        raise ValueError("Magic invalido: nao e container ACS1")
    mode = blob[4]
    original_len = struct.unpack_from("<I", blob, 5)[0]
    payload_len = struct.unpack_from("<I", blob, 9)[0]
    if len(blob) != 13 + payload_len:
        raise ValueError("Tamanho de payload inconsistente no container")
    payload = blob[13:]
    return mode, original_len, payload


def main() -> int:
    parser = argparse.ArgumentParser(description="Restaura arquivo salvo por send_file_uart.py (container ACS1).")
    parser.add_argument("--in", dest="input_path", required=True, help="Arquivo de entrada (.bin container)")
    parser.add_argument("--out", dest="output_path", required=True, help="Arquivo restaurado")
    parser.add_argument("--verify", dest="verify_path", help="Compara com arquivo original esperado")
    args = parser.parse_args()

    blob = Path(args.input_path).read_bytes()
    mode, original_len, payload = parse_container(blob)

    if mode == MODE_RAW:
        restored = payload
    elif mode == MODE_ARITH:
        restored = decode_adaptive(payload, original_len)
    else:
        raise ValueError(f"Modo desconhecido no container: {mode}")

    if len(restored) != original_len:
        raise ValueError(f"Tamanho restaurado inconsistente: {len(restored)} != {original_len}")

    Path(args.output_path).write_bytes(restored)
    print(f"[OK] Restaurado {len(restored)} bytes em {args.output_path}")

    if args.verify_path:
        expected = Path(args.verify_path).read_bytes()
        if restored != expected:
            print("[ERRO] VERIFY falhou: conteudo diferente do esperado", file=sys.stderr)
            return 2
        print("[OK] VERIFY passou: restaurado == original")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
