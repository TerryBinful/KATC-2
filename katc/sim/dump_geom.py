"""Dump junction internal-lane geometry to detect physically crossing paths."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import traci
traci.start(["sumo","-c","intersection.sumocfg","--start","--quit-on-end"])

# For each controlled link on the major W approach, print entry+exit points.
links = traci.trafficlight.getControlledLinks("C")
for idx, tuples in enumerate(links):
    for (inLane, outLane, viaLane) in tuples:
        if inLane.startswith("W_C"):
            try:
                in_shape = traci.lane.getShape(inLane)
                out_shape = traci.lane.getShape(outLane)
                # entry = last point of in-lane, exit = first point of out-lane
                entry = in_shape[-1]
                exit_ = out_shape[0]
                print(f"link {idx}: {inLane} -> {outLane}")
                print(f"    enters junction at ({entry[0]:.1f},{entry[1]:.1f})  exits at ({exit_[0]:.1f},{exit_[1]:.1f})")
            except Exception as e:
                print(f"link {idx}: {inLane} -> {outLane}  [{e}]")
traci.close()
