from st3215 import ST3215
import time

s = ST3215('/dev/ttyUSB1')

#read al servo position
for i in range(1,10):
    print('servo',i,':',s.ReadPosition(i))
    time.sleep(0.1)


SERVO_IDS = [1, 2, 4, 5, 7, 8,10,11]

#              S1    S2    S4    S5    S7    S8
HOME      = [2048, 2760, 2048, 1200, 2048, 1200,2048, 2760]

UAV_STEP1 = [2048, 2048, 2048, 2048, 2048, 2048,2048, 2048]  
UAV_STEP2 = [ 1684, 2032, 2378, 2134, 2347, 2174, 1740, 2062]  
UAV_STEP3 = [ 1684, 2971, 2378, 997, 2347, 1210, 1740, 2997] 
UAV_HOME= [ 1092, 2973, 3016, 1122, 2982, 1226,1096, 2996]  
UAV_STEPS = [UAV_STEP1,UAV_STEP2,UAV_STEP3,UAV_HOME]


UGV_STEP1 = [ 1684, 2971, 2378, 997, 2347, 1210, 1740, 2997]  
UGV_STEP2 = [ 1684, 2032, 2378, 2134, 2347, 2174, 1740, 2062]  
UGV_STEP3 = [ 2048, 2048, 2048, 2048, 2048, 2048,2048, 2048] 
UGV_HOME = [2048, 2875, 2048, 1217, 2048, 1327, 2048, 2865] 


UGV_STEPS = [ UGV_HOME]

# UAV_STEP4, UGV_STEP1,UGV_STEP2,UGV_STEP3,
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

def run_sequence(name, steps):
    print(f"\n--- Commanding {name} Sequence ---")
    
    # Setup servos for movement
    for sid in SERVO_IDS:
        s.StartServo(sid)
        s.SetMode(sid, 0)
        s.SetSpeed(sid, 500)
        s.SetAcceleration(sid, 50)
    
    # Execute each step sequentially
    for step_num, step_positions in enumerate(steps, 1):
        print(f"\n--- {name} Step {step_num}/{len(steps)}: {step_positions} ---")
        
        # Send command for this step
        for sid, pos in zip(SERVO_IDS, step_positions):
            s.WritePosition(sid, pos)
        
        # Monitor until settled
        for i in range(4):
            time.sleep(1.0)
            curr = get_current_positions()
            status, max_delta = get_status(curr)
            print(f"  [{i+1}/4] Status: {status} (Max Error: {max_delta}) | Positions: {curr}")
    
    print(f"\nAll {name} steps complete.")

run_sequence("UGV", UGV_STEPS)
# run_sequence("UAV", UAV_STEPS)
# run_sequence("UGV", UGV_STEPS)

