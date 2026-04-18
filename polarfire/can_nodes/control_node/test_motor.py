#!/usr/bin/env python3
"""
GR-00 Motor Test — Step 1: Get a response from RS03 (CAN ID 1)
Tries all known enable strategies and prints raw bytes of any response.
Run on PolarFire: python3 test_motor.py
Motor must be powered (49V) before running.
"""
 
import can
import time
import struct
 
MOTOR_ID = 1
MASTER_ID = 0xFD
IFACE = "can0"
LISTEN_TIMEOUT = 0.3  # seconds to wait for response after each frame
 
bus = can.interface.Bus(channel=IFACE, bustype="socketcan")
 
def rx(timeout=LISTEN_TIMEOUT):
    """Listen for responses, return list of (arb_id, is_extended, data_bytes)."""
    responses = []
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = bus.recv(timeout=max(0, deadline - time.time()))
        if msg:
            responses.append((msg.arbitration_id, msg.is_extended_id, bytes(msg.data)))
    return responses
 
def print_responses(tag, responses):
    if not responses:
        print(f"  [{tag}] no response")
    for arb_id, is_ext, data in responses:
        id_type = "EXT" if is_ext else "STD"
        hex_data = " ".join(f"{b:02X}" for b in data)
        print(f"  [{tag}] {id_type} ID=0x{arb_id:08X}  data: {hex_data}")
 
def send(arb_id, data, is_extended=False, label=""):
    msg = can.Message(
        arbitration_id=arb_id,
        data=data,
        is_extended_id=is_extended
    )
    try:
        bus.send(msg)
        id_str = f"0x{arb_id:08X}" if is_extended else f"0x{arb_id:03X}"
        ext_str = "EXT" if is_extended else "STD"
        hex_data = " ".join(f"{b:02X}" for b in data)
        print(f"  TX [{label}] {ext_str} ID={id_str}  data: {hex_data}")
    except can.CanError as e:
        print(f"  TX ERROR [{label}]: {e}")
        return False
    return True
 
print("=" * 60)
print(f"GR-00 Motor Test — RS03 CAN ID {MOTOR_ID}")
print("=" * 60)
 
# ── STRATEGY A: MIT mode, standard CAN ID ──────────────────────────
# Source: C++ RobStride_Motor_MIT_Enable(), matches original docs
print("\n[A] MIT Enable — STD ID = motor_id, data = FF FF FF FF FF FF FF FC")
enable_data_fc = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
send(MOTOR_ID, enable_data_fc, is_extended=False, label="A-enable")
resp_a = rx()
print_responses("A", resp_a)
 
# ── STRATEGY B: Extended ID, 0x03 comm type (non-MIT enable) ───────
# Source: C++ Enable_Motor() non-MIT path
# ExtId = (0x03 << 24) | (MASTER_ID << 8) | MOTOR_ID
print("\n[B] Non-MIT Enable — EXT ID = (0x03<<24)|(0xFD<<8)|motor_id, data = 00s")
ext_id_b = (0x03 << 24) | (MASTER_ID << 8) | MOTOR_ID
send(ext_id_b, bytes(8), is_extended=True, label="B-enable")
resp_b = rx()
print_responses("B", resp_b)
 
# ── STRATEGY C: Extended ID 0x200 + motor_id (Seeed wiki) ──────────
print("\n[C] Seeed-wiki Enable — EXT ID = 0x200 + motor_id, data = FF*8")
ext_id_c = 0x200 + MOTOR_ID
send(ext_id_c, bytes([0xFF]*8), is_extended=True, label="C-enable")
resp_c = rx()
print_responses("C", resp_c)
 
# ── STRATEGY D: Same as A but also try motor IDs 2-8 ───────────────
# In case the motor's CAN ID was factory-reset to something else
print("\n[D] MIT Enable broadcast — trying IDs 1-8 to find any live motor")
found_ids = []
for mid in range(1, 9):
    send(mid, enable_data_fc, is_extended=False, label=f"D-id{mid}")
    r = rx(timeout=0.15)
    if r:
        found_ids.append((mid, r))
        print_responses(f"D-id{mid}", r)
    else:
        print(f"  [D-id{mid}] no response")
 
# ── STRATEGY E: Request motor state (CommType 0x02) ─────────────────
# This is a read request, motor should reply even if not enabled
print("\n[E] Motor state request — EXT ID = (0x02<<24)|(0xFD<<8)|motor_id")
ext_id_e = (0x02 << 24) | (MASTER_ID << 8) | MOTOR_ID
send(ext_id_e, bytes(8), is_extended=True, label="E-statereq")
resp_e = rx()
print_responses("E", resp_e)
 
# ── STRATEGY F: Get ID request (CommType 0x00) ──────────────────────
print("\n[F] Get device ID — EXT ID = (0x00<<24)|(0xFD<<8)|motor_id")
ext_id_f = (0x00 << 24) | (MASTER_ID << 8) | MOTOR_ID
send(ext_id_f, bytes(8), is_extended=True, label="F-getid")
resp_f = rx()
print_responses("F", resp_f)
 
print("\n" + "=" * 60)
print("SUMMARY")
print("=" * 60)
results = {
    "A (MIT STD FC)": resp_a,
    "B (EXT 0x03)":   resp_b,
    "C (EXT 0x200)":  resp_c,
    "D (scan 1-8)":   found_ids,
    "E (state req)":  resp_e,
    "F (get ID)":     resp_f,
}
any_response = False
for name, resp in results.items():
    if name == "D (scan 1-8)":
        got = bool(found_ids)
    else:
        got = bool(resp)
    status = "✅ GOT RESPONSE" if got else "❌ no response"
    print(f"  {name}: {status}")
    if got:
        any_response = True
 
if not any_response:
    print("\n⚠️  No response from any strategy.")
    print("   Check: 49V power on? CAN-H/CAN-L correct? Termination?")
else:
    print("\n✅ At least one strategy worked — paste output above to proceed.")
 
bus.shutdown()
 