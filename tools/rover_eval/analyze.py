import csv, json, glob, os, math
R = 0.145
rows = []
for d in sorted(glob.glob("/workspace/runs/*/")):
    name = os.path.basename(d.rstrip("/"))
    if name.startswith(("smoke","prev","test","v","ctest","jit","plane","tiles","flat","mesh","zt","camtest","btest","diag")) or not os.path.exists(d+"run.csv"): continue
    cfg = json.load(open(d+"config.json"))
    r = list(csv.DictReader(open(d+"run.csv")))
    if len(r) < 10: continue
    f = lambda k: [float(a[k]) for a in r]
    x,y,z,roll,pitch,wm,tau,qL,qR,cmd = f("x"),f("y"),f("z"),f("roll"),f("pitch"),f("wheel_mean"),f("tau"),f("qL"),f("qR"),f("cmd_vel")
    rms = lambda v: math.sqrt(sum(a*a for a in v)/len(v))
    dt = 1/60
    vx = [(x[i+1]-x[i])/dt for i in range(len(x)-1)]
    slip = [1-abs(vx[i])/(abs(wm[i])*R) for i in range(len(vx)) if abs(wm[i])*R > 0.2]
    art = [a-b for a,b in zip(qL,qR)]; cm = [a+b for a,b in zip(qL,qR)]
    log = open(d+"log.txt").read(); end = [l for l in log.splitlines() if "SESSION-ENDED" in l]
    rows.append({"run":name,"model":cfg["model"],"mode":cfg["mode"],"speed":cfg["speed_rad_s"],"lane":int(cfg["lane_keeping"]),
        "t_s":round(len(r)*dt,1),"dist_m":round(x[-1]-x[0],1),"y_drift_m":round(max(abs(a) for a in y),2),
        "sigma_z_cm":round(100*math.sqrt(sum((a-sum(z)/len(z))**2 for a in z)/len(z)),1),
        "roll_rms":round(rms(roll),2),"roll_max":round(max(abs(a) for a in roll),1),
        "pitch_rms":round(rms(pitch),2),"pitch_max":round(max(abs(a) for a in pitch),1),
        "artic_rms_deg":round(math.degrees(rms(art)),2),"cm_rms_deg":round(math.degrees(rms(cm)),2),
        "tau_sat_pct":round(100*sum(abs(a)>=cfg["tau_max"]-0.01 for a in tau)/len(tau),1) if cfg["tau_max"]>0 else 0.0,
        "slip_pct":round(100*sum(slip)/len(slip),1) if slip else 0.0,
        "rollover":int(any(abs(a)>45 for a in roll)),"end":end[0].split("reason=")[1].split()[0] if end else "?"})
keys = list(rows[0].keys())
with open("/workspace/runs/metrics.csv","w",newline="") as fh:
    w=csv.DictWriter(fh,keys); w.writeheader(); w.writerows(rows)
with open("/workspace/runs/metrics.md","w") as fh:
    fh.write("| "+" | ".join(keys)+" |\n|"+"---|"*len(keys)+"\n")
    for rw in rows: fh.write("| "+" | ".join(str(rw[k]) for k in keys)+" |\n")
print(open("/workspace/runs/metrics.md").read())
