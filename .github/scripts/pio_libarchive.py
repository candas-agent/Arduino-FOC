#!/usr/bin/env python3
"""
Compiles SimpleFOC examples with PlatformIO in 3 ways and compares them (library archiving check).

  docs     : library.json with only {"build": {"libArchive": false}} + lib_archive = false in the project
             (what the documentation asks today)
  registry : no library.json (the PlatformIO registry package 2.4.0 has none) and no lib_archive
  fixed    : the complete library.json of this branch and no lib_archive in the project

For each build: success, flash and RAM, and which object file provides each hardware API function
(read from the linker map: generic_mcu.cpp means the weak, empty default).
The job fails if "fixed" does not compile where "docs" does, or links a hardware function from another
object file than "docs".

  build:      pio_libarchive.py --name esc1 --platform ststm32 --board disco_b_g431b_esc1 \
                  --sketches B_G431B_ESC1 [--exclude=a,b] [--flags="-DX"] [--lib-deps="a, b"] --out results
  summarize:  pio_libarchive.py --summarize results
"""
import argparse
import glob
import json
import os
import re
import shutil
import subprocess
import sys

VARIANTS = ["docs", "registry", "fixed"]
HW_FUNCTIONS = {
    "_configure1PWM", "_configure2PWM", "_configure3PWM", "_configure4PWM", "_configure6PWM",
    "_writeDutyCycle1PWM", "_writeDutyCycle2PWM", "_writeDutyCycle3PWM", "_writeDutyCycle4PWM",
    "_writeDutyCycle6PWM", "_configureADCInline", "_configureADCLowSide", "_readADCVoltageInline",
    "_readADCVoltageLowSide", "_driverSyncLowSide", "_startADC3PinConversionLowSide",
}
REPO = os.getcwd()


def make_variants(tmp):
    libs = {}
    for v in VARIANTS:
        dst = os.path.join(tmp, v, "SimpleFOC")
        if os.path.isdir(dst):
            shutil.rmtree(dst)
        shutil.copytree(REPO, dst, ignore=shutil.ignore_patterns(".git", ".github", ".pio", "results*"))
        manifest = os.path.join(dst, "library.json")
        if v == "docs":
            with open(manifest, "w") as f:
                json.dump({"build": {"libArchive": False}}, f)
        elif v == "registry":
            os.remove(manifest)
        libs[v] = dst
    return libs


def find_sketches(names, exclude):
    every = sorted(glob.glob(os.path.join(REPO, "examples", "**", "*.ino"), recursive=True))
    stem = lambda p: os.path.splitext(os.path.basename(p))[0]
    if names == ["**"]:
        return [p for p in every if stem(p) not in exclude]
    out = []
    for n in names:
        if "/" in n:
            out.append(os.path.join(REPO, n))
        else:
            out += [p for p in every if stem(p) == n]
    return out


def demangled_name(section_symbol):
    # _Z19_writeDutyCycle6PWMfffP10PhaseStatePv -> _writeDutyCycle6PWM
    m = re.match(r"_Z(\d+)(.*)", section_symbol)
    if not m:
        return section_symbol
    return m.group(2)[: int(m.group(1))]


def parse_map(path):
    """hardware function -> object file that provides it (kept sections of the memory map only)"""
    with open(path, errors="replace") as f:
        text = f.read()
    i = text.find("Linker script and memory map")
    text = text[i:] if i >= 0 else text
    found = {}
    pat = re.compile(r"^ \.(?:text|literal|iram1)\.(_Z\w+)\s*\n?\s+0x[0-9a-fA-F]+\s+0x([0-9a-fA-F]+)\s+(\S.*)$", re.M)
    for sym, size, obj in pat.findall(text):
        name = demangled_name(sym)
        if name in HW_FUNCTIONS and int(size, 16) > 0:
            obj = obj.strip().replace("\\", "/")
            m = re.search(r"\(([^()]+)\)$", obj)  # archive member: libX.a(file.o)
            found[name] = m.group(1) if m else os.path.basename(obj)
    return found


def build(sketch, lib, variant, a, work):
    bdir = os.path.join(work, "build")
    if os.path.isdir(bdir):
        shutil.rmtree(bdir)
    os.makedirs(bdir)
    # linker map in the build folder of the env (some platforms, e.g. STM32, already ask for this one)
    cmd = ["pio", "ci", sketch, "--lib", lib, "--board", a.board, "--keep-build-dir", "--build-dir", bdir,
           "-O", f"platform={a.platform}", "-O", f"build_flags={a.flags} -Wl,-Map,$BUILD_DIR/firmware.map"]
    if variant == "docs":
        cmd += ["-O", "lib_archive=false"]
    if a.lib_deps:
        cmd += ["-O", f"lib_deps={a.lib_deps}"]
    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    log = p.stdout
    num = lambda pat: int(m.group(1)) if (m := re.search(pat, log)) else None
    res = {"ok": p.returncode == 0,
           "flash": num(r"Flash:.*?used (\d+) bytes"), "ram": num(r"RAM:.*?used (\d+) bytes"),
           "hw": {}}
    maps = glob.glob(os.path.join(bdir, ".pio", "build", "*", "firmware.map"))
    if maps:
        res["hw"] = parse_map(maps[0])
    if not res["ok"]:
        res["error"] = "\n".join(l for l in log.splitlines() if "error" in l.lower())[-1500:]
    return res


def run_builds(a):
    work = os.path.join(os.environ.get("RUNNER_TEMP", "/tmp"), "pio_libarchive")
    libs = make_variants(work)
    sketches = find_sketches([s.strip().removesuffix(".ino") for s in a.sketches.split(",") if s.strip()],
                             {s.strip() for s in a.exclude.split(",") if s.strip()})
    results = []
    for sk in sketches:
        rel = os.path.relpath(sk, REPO).replace("\\", "/")
        row = {"name": a.name, "board": a.board, "sketch": rel}
        for v in VARIANTS:
            print(f"::group::{a.name} {rel} [{v}]", flush=True)
            row[v] = build(sk, libs[v], v, a, work)
            print(json.dumps(row[v]), flush=True)
            print("::endgroup::", flush=True)
        results.append(row)
    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, f"{a.name}.json"), "w") as f:
        json.dump(results, f, indent=1)
    errors = [r for r in results if r["docs"]["ok"] and (not r["fixed"]["ok"] or r["fixed"]["hw"] != r["docs"]["hw"])]
    for r in errors:
        print(f"::error::{r['name']} {r['sketch']}: fixed differs from docs: {r['fixed']} vs {r['docs']}")
    return 1 if errors else 0


def fmt(r):
    if not r["ok"]:
        return "FAILED"
    return f"{r['flash']} / {r['ram']}" if r["flash"] is not None else "ok"


def summarize(folder):
    rows = []
    for f in sorted(glob.glob(os.path.join(folder, "**", "*.json"), recursive=True)):
        with open(f) as fh:
            rows += json.load(fh)
    print("## PlatformIO: library archiving\n")
    print("Flash / RAM in bytes. **hw**: object files providing the hardware API functions "
          "(`generic` = the weak empty defaults of generic_mcu.cpp).\n")
    print("| Board | Sketch | docs (lib_archive = false) | registry (no library.json) | registry hw | fixed (complete library.json) | fixed - docs flash | hw same as docs |")
    print("|---|---|---|---|---|---|---|---|")
    n_bad_registry = n_err = 0
    for r in rows:
        d, g, x = r["docs"], r["registry"], r["fixed"]
        generic = sorted(k for k, o in g["hw"].items() if "generic_mcu" in o and "generic_mcu" not in d["hw"].get(k, ""))
        if generic:
            n_bad_registry += 1
        delta = (x["flash"] - d["flash"]) if (d["ok"] and x["ok"] and d["flash"] is not None and x["flash"] is not None) else ""
        same = "yes" if x["hw"] == d["hw"] else ("n/a" if not d["ok"] else "**NO**")
        if d["ok"] and same != "yes":
            n_err += 1
        reg_hw = ("**generic: " + ", ".join(generic) + "**") if generic else "same"
        print(f"| {r['board']} | {os.path.basename(r['sketch'])} | {fmt(d)} | {fmt(g)} | {reg_hw} | {fmt(x)} | {delta} | {same} |")
    print(f"\n{len(rows)} sketches. Registry package links empty generic hardware functions in **{n_bad_registry}** of them. "
          f"Fixed library.json differs from lib_archive = false in **{n_err}**.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--summarize", metavar="FOLDER")
    ap.add_argument("--name")
    ap.add_argument("--platform")
    ap.add_argument("--board")
    ap.add_argument("--sketches", default="")
    ap.add_argument("--exclude", default="")
    ap.add_argument("--flags", default="")
    ap.add_argument("--lib-deps", default="")
    ap.add_argument("--out", default="results")
    a = ap.parse_args()
    if a.summarize:
        summarize(a.summarize)
        return 0
    return run_builds(a)


if __name__ == "__main__":
    sys.exit(main())
