from st3215 import ST3215
import time

s = ST3215('/dev/ttyUSB0')

SERVO_IDS = [1, 2, 4, 5, 7, 8, 10, 11]

# ── Single snapshot ───────────────────────────────────────────────────────────

def read_all_loads():
    print(f"\n{'Servo':<8} {'Load %':>8} {'Voltage V':>10} {'Temp °C':>9} {'Current mA':>11}")
    print("-" * 50)
    for sid in SERVO_IDS:
        load    = s.ReadLoad(sid)
        voltage = s.ReadVoltage(sid)
        temp    = s.ReadTemperature(sid)
        current = s.ReadCurrent(sid)
        time.sleep(0.01)

        load_str    = f"{load:.1f}"    if load    is not None else "ERR"
        voltage_str = f"{voltage:.1f}" if voltage is not None else "ERR"
        temp_str    = f"{temp}"        if temp    is not None else "ERR"
        current_str = f"{current:.0f}" if current is not None else "ERR"

        flag = ""
        if load is not None and load > 50:
            flag = "  << HIGH LOAD"
        elif load is not None and load > 30:
            flag = "  < elevated"

        print(f"  S{sid:<6} {load_str:>8} {voltage_str:>10} {temp_str:>9} {current_str:>11}{flag}")


# ── Continuous monitor — watch load while you move the robot by hand ──────────

def monitor_loads(duration_s=30, interval_s=0.5):
    print(f"\n=== Live Load Monitor ({duration_s}s, every {interval_s}s) ===")
    print("Move each joint by hand to feel resistance vs load reading.\n")
    print(f"{'Time':>6}  " + "  ".join(f"S{sid:>2}" for sid in SERVO_IDS))
    print("-" * 70)

    start = time.time()
    while time.time() - start < duration_s:
        elapsed = time.time() - start
        loads = []
        for sid in SERVO_IDS:
            load = s.ReadLoad(sid)
            loads.append(f"{load:.0f}%" if load is not None else "ERR")
            time.sleep(0.005)

        row = f"{elapsed:>5.1f}s  " + "  ".join(f"{l:>5}" for l in loads)

        # Highlight if servo 8 load stands out
        s8_idx = SERVO_IDS.index(8)
        try:
            s8_val = float(loads[s8_idx].replace('%',''))
            others = [float(loads[i].replace('%','')) for i in range(len(SERVO_IDS))
                      if i != s8_idx and loads[i] != 'ERR']
            avg_others = sum(others) / len(others) if others else 0
            if s8_val > avg_others + 15:
                row += "  << S8 HIGH"
        except Exception:
            pass

        print(row)
        time.sleep(interval_s)


# ── Run ───────────────────────────────────────────────────────────────────────

print("=== Servo Load Diagnostic ===")
print("\n-- Snapshot (static, at rest) --")
read_all_loads()

print("\n\n-- Starting live monitor (hold robot still, then move each joint) --")
monitor_loads(duration_s=30, interval_s=0.5)