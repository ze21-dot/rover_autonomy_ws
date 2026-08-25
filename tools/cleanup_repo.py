"""Repo cleanup before merge. Run from repo root:  python tools/cleanup_repo.py
1. runs/*/log.txt -> drop everything before the first '[cfg]' line (Isaac startup banner / machine info)
2. README wording -> neutral ('GPU workstation'), no host/provider references
3. scan all text files for forbidden tokens and report (does not auto-delete)
"""
import re, glob, os, sys, io

ROOT = os.getcwd()
FORBIDDEN = re.compile(r"vast\.?ai|vastai|GPU workstation|claude|anthropic|chatgpt|openai|\bgpt\b|\bpaper\b|\bfair\b|exhibition|\bfuar\b|captain|\bICRA\b|\bIROS\b|Tsinghua|Beijing|\bURC\b|\bERC\b|Zeynep|competition", re.I)
REPL = [
    (re.compile(r"GPU workstation", re.I), "GPU workstation"),
    (re.compile(r"team standard", re.I), "team standard"),
    (re.compile(r"Quick start \(GPU workstation\)", re.I), "Quick start (GPU workstation)"),
    (re.compile(r"on the workstation", re.I), "on the workstation"),
    (re.compile(r"in any report", re.I), "in any report"),
    (re.compile(r"report-grade", re.I), "report-grade"),
]

# 1. logs
n = 0
for f in glob.glob("runs/*/log.txt"):
    lines = io.open(f, encoding="utf-8", errors="ignore").read().splitlines()
    idx = next((i for i, l in enumerate(lines) if l.startswith("[cfg]")), None)
    if idx is not None and idx > 0:
        keep = [l for l in lines[idx:] if not l.startswith("2026-") or "Error" in l or "SESSION" in l]
        io.open(f, "w", encoding="utf-8").write("\n".join(keep) + "\n"); n += 1
print(f"logs trimmed: {n}")

# 2. wording
for f in glob.glob("**/*.md", recursive=True) + glob.glob("**/*.sh", recursive=True) + glob.glob("**/*.py", recursive=True):
    s = io.open(f, encoding="utf-8", errors="ignore").read(); s0 = s
    for pat, rep in REPL: s = pat.sub(rep, s)
    if s != s0: io.open(f, "w", encoding="utf-8").write(s); print("reworded:", f)

# 3. scan
hits = []
for f in glob.glob("**/*", recursive=True):
    if os.path.isdir(f) or ".git" in f.split(os.sep) or f.endswith((".mp4", ".png", ".jpg", ".STL", ".stl", ".usd", ".usda")): continue
    try: s = io.open(f, encoding="utf-8", errors="ignore").read()
    except Exception: continue
    for i, l in enumerate(s.splitlines(), 1):
        if FORBIDDEN.search(l): hits.append(f"{f}:{i}: {l.strip()[:100]}")
print("\nforbidden-token hits:", len(hits))
for h in hits: print("  ", h)
