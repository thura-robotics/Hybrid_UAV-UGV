from st3215 import ST3215
import time

s = ST3215('/dev/ttyUSB0')

#read al servo position
for i in range(1,10):
    print('servo',i,':',s.ReadPosition(i))
    time.sleep(0.1)


SERVO_IDS = [1, 2, 4, 5, 7, 8]

#              S1    S2    S4    S5    S7    S8
HOME      = [2048, 2760, 2048, 1200, 2048, 1200]

UAV_STEP1 = [2048, 2048, 2048, 2048, 2048, 2048]  
UAV_STEP2 = [ 900, 2048, 2900, 2048, 2900, 2048]  
UAV_STEP3 = [ 900, 2900, 2900, 1000, 2900, 1000]  
UAV_STEPS = [UAV_STEP1, UAV_STEP2, UAV_STEP3]

UGV_STEP1 = [ 900, 2048, 2900, 2048, 2900, 2048]  
UGV_STEP2 = [ 2048, 2048, 2048, 2048, 2048, 2048]  
UGV_STEP3 = [ 2048, 2900, 2082, 1157, 2023, 1011]  
UGV_STEPS = [UGV_STEP1, UGV_STEP2, UGV_STEP3]

# Threshold for matching positions (ticks)
MATCH_THRESHOLD = 17

def get_current_positions():
    positions = []
    for sid in SERVO_IDS:
        pos = s.ReadPosition(sid)
        if pos is None:
            return None
        positions.append(pos)
    return positions

def get_status(current_pos):
    if current_pos is None:
        return "ERROR: Connection Lost", 0
    
    best_match = "TRANSITIONING / UNKNOWN"
    min_max_delta = 9999

    def check_match(target_pos, name):
        deltas = [abs(c - t) for c, t in zip(current_pos, target_pos)]
        max_delta = max(deltas)
        if max_delta <= MATCH_THRESHOLD:
            return name, max_delta
        return None, max_delta

    # Check HOME
    res, d = check_match(HOME, "HOME")
    if res: return res, d
    if d < min_max_delta: min_max_delta = d

    # Check UAV
    for i, step in enumerate(UAV_STEPS, 1):
        res, d = check_match(step, f"UAV_STEP{i}")
        if res: return res, d
        if d < min_max_delta: min_max_delta = d
            
    # Check UGV
    for i, step in enumerate(UGV_STEPS, 1):
        res, d = check_match(step, f"UGV_STEP{i}")
        if res: return res, d
        if d < min_max_delta: min_max_delta = d
            
    return "TRANSITIONING / UNKNOWN", min_max_delta

print("\n--- Commanding ---")
print(f"Target: {UAV_STEP2}")

# Setup servos for movement
for sid in SERVO_IDS:
    s.StartServo(sid)
    s.SetMode(sid, 0)
    s.SetSpeed(sid, 300)
    s.SetAcceleration(sid, 50)

# Send command
for sid, pos in zip(SERVO_IDS, UGV_STEP3):
    s.WritePosition(sid, pos)

print("Moving... checking status...")
# Monitor progress for 10 seconds
for i in range(10):
    time.sleep(1.0)
    curr = get_current_positions()
    status, max_delta = get_status(curr)
    print(f"[{i+1}/10] Status: {status} (Max Error: {max_delta}) | Positions: {curr}")
   
print("\nTask complete.")

