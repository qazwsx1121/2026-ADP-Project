import json, os, sys
d = json.load(open(os.path.expanduser("~/forza_ws/race_stack/stack_master/maps/berlin/global_waypoints.json")))
w = d["global_traj_wpnts_iqp"]["wpnts"]
lo, hi = float(sys.argv[1]), float(sys.argv[2])
for p in w:
    if lo <= p["s_m"] <= hi and round(p["s_m"] * 10) % 5 == 0:
        k = p["kappa_radpm"]
        print("s=%5.1f x=%6.2f y=%6.2f vx=%5.2f scaled=%4.2f kappa=%6.2f R=%5.2f dL=%4.2f dR=%4.2f" % (
            p["s_m"], p["x_m"], p["y_m"], p["vx_mps"], 0.5 * p["vx_mps"], k, 1 / abs(k) if abs(k) > 1e-3 else 99, p["d_left"], p["d_right"]))
