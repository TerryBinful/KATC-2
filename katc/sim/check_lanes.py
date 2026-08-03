"""Print exactly which movement each controlled link serves, to catch flips."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import traci
import katc_sumo as K

traci.start(["sumo","-c","intersection.sumocfg","--start","--quit-on-end"])
links = traci.trafficlight.getControlledLinks(K.TLS_ID)
print(f"Total controlled links: {len(links)}\n")
dest_role = {
    ("W_C","C_E"):"MAJOR THROUGH", ("W_C","C_S"):"MAJOR LEFT", ("W_C","C_N"):"major right",
    ("E_C","C_W"):"MAJOR THROUGH", ("E_C","C_N"):"MAJOR LEFT", ("E_C","C_S"):"major right",
    ("N_C","C_S"):"minor through", ("N_C","C_E"):"minor left", ("N_C","C_W"):"minor right",
    ("S_C","C_N"):"minor through", ("S_C","C_W"):"minor left", ("S_C","C_E"):"minor right",
}
print("Link | fromLane | movement")
for idx, tuples in enumerate(links):
    for (inLane, outLane, _v) in tuples:
        ie = inLane.rsplit("_",1)[0]; il = inLane.rsplit("_",1)[1]
        oe = outLane.rsplit("_",1)[0]
        role = dest_role.get((ie,oe), f"{ie}->{oe}")
        print(f"  {idx:2d} | {ie} lane{il} | {role}")

# Now show what MOVEMENT_DEF resolves and whether through/left got the right links
mv, n, perm = K.build_movement_links(traci)
print("\nNEMA movement -> resolved links (2/6 should be THROUGH, 1/5 LEFT):")
role_of = {1:"major LEFT",2:"major THROUGH",3:"minor left",4:"minor through",
           5:"major LEFT",6:"major THROUGH",7:"minor left",8:"minor through"}
for m in sorted(mv):
    print(f"  NEMA {m} ({role_of[m]}): links {mv[m]}")
print(f"  permissive (right) links: {perm}")
traci.close()
