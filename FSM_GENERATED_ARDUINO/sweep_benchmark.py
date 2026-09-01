#!/usr/bin/env python3
"""
sweep_benchmark.py
===================
Prosiruje logiku iz auto_benchmark.py da automatski build-uje/flash-uje sve
fajlove generisane fsm_generator.py --sweep, cita rezultate (Flash, RAM, min/
avg/max cikli) sa serijskog porta, i sklapa ih u DUGI (long) CSV format:

    platform, pattern, param, param_value, metric, value

pogodan direktno za curve_analysis.py (jedan red po metrici, ne jedan red sa
svim metrikama u kolonama - lakse za grupisanje/fitovanje po (platform,pattern)).

Preduslovi:
1. Vec pokrenut: python fsm_generator.py --sweep
   (generise output/generated_fsm/*.cpp, output/sweep_manifest.csv,
    output/platformio_sweep_avr.ini, output/platformio_sweep_esp32.ini)
2. PlatformIO CLI dostupan u PATH (pio), Arduino/ESP32 board povezan.

Napomena: skripta NE modifikuje platformio.ini na isti "prepisi ceo fajl" nacin
kao original auto_benchmark.py, jer sweep_manifest.csv vec ima gotov env po
fajlu - dovoljno je promeniti build_src_filter i default_envs.

ISPRAVKA u odnosu na prvobitnu verziju: build_and_upload() je ranije hvatao
Flash/RAM po REDOSLEDU pojavljivanja teksta "used X bytes from Y bytes" u
pio izlazu (prvi match -> flash, drugi -> ram). PlatformIO CLI po defaultu
ispisuje RAM red PRE Flash reda, pa je to davalo ZAMENJENE vrednosti. Sada se
hvata eksplicitno po labeli ("RAM:" / "Flash:"), sto je nezavisno od redosleda
u izlazu.

Upotreba:
    python sweep_benchmark.py --platform avr --port COM3
    python sweep_benchmark.py --platform esp32 --port /dev/ttyUSB0
"""

import argparse
import csv
import os
import re
import subprocess
import time

import serial
import serial.tools.list_ports

BAUD_RATE = 9600


def find_port(explicit_port=None):
    if explicit_port:
        return explicit_port
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = p.description.lower()
        if "arduino" in desc or "ch340" in desc or "usb serial" in desc or "cp210" in desc or "ftdi" in desc:
            return p.device
    if ports:
        return ports[0].device
    return None


def load_manifest(path="output/sweep_manifest.csv"):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def set_active_env(ini_template_path, env_name, working_ini="platformio.ini"):
    """Cita generisani ini (koji ima SVE env-ove za tu platformu) i upisuje
    working platformio.ini sa default_envs postavljenim na trazeni env - ovo
    izbegava potrebu da regenerisemo ceo ini string u Python-u svaki put
    (za razliku od auto_benchmark.py), samo prepisujemo default_envs liniju."""
    with open(ini_template_path) as f:
        content = f.read()
    content = re.sub(r"default_envs\s*=.*", f"default_envs = {env_name}", content, count=1)
    with open(working_ini, "w") as f:
        f.write(content)


def build_and_upload(env_name):
    subprocess.run("pio run -t clean", shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    res = subprocess.run(f"pio run -e {env_name} -t upload", shell=True,
                          capture_output=True, text=True)
    out = res.stdout + res.stderr
    if res.returncode != 0:
        print(f"-> GRESKA pri build/upload za {env_name}:")
        print(out[-800:])
        return None, None

    # ISPRAVKA: hvatanje po labeli ("RAM:" / "Flash:"), ne po redosledu pojavljivanja
    # u izlazu - pio obicno stampa RAM red PRE Flash reda, pa oslanjanje na
    # "prvi match = flash, drugi = ram" daje zamenjene vrednosti.
    ram_match = re.search(r"RAM:.*?used (\d+) bytes", out)
    flash_match = re.search(r"Flash:.*?used (\d+) bytes", out)

    ram_bytes = int(ram_match.group(1)) if ram_match else None
    flash_bytes = int(flash_match.group(1)) if flash_match else None

    if flash_bytes is None or ram_bytes is None:
        print(f"-> UPOZORENJE: nisam uspela da parsiram Flash/RAM iz pio izlaza za {env_name}.")
        print("   Proveri da li se format pio izlaza promenio (poslednjih 800 karaktera):")
        print(out[-800:])

    return flash_bytes, ram_bytes


def read_benchmark_serial(port, timeout=8):
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=timeout)
        time.sleep(2.0)
        ser.reset_input_buffer()
        ser.write(b'b')
        ser.flush()

        result = {"n_states": None, "n_events": None, "min_c": None, "max_c": None, "avg_c": None}
        start_t = time.time()
        while time.time() - start_t < timeout:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            m = re.search(r"N_STATES:\s*(\d+)", line)
            if m:
                result["n_states"] = int(m.group(1))
            m = re.search(r"N_EVENTS:\s*(\d+)", line)
            if m:
                result["n_events"] = int(m.group(1))
            m = re.search(r"Min cycles:\s*(\d+)", line)
            if m:
                result["min_c"] = int(m.group(1))
            m = re.search(r"Max cycles:\s*(\d+)", line)
            if m:
                result["max_c"] = int(m.group(1))
            m = re.search(r"Avg cycles:\s*(\d+)", line)
            if m:
                result["avg_c"] = int(m.group(1))
            if all(v is not None for v in result.values()):
                break
        ser.close()
        return result
    except Exception as e:
        print(f"-> GRESKA na serijskom portu: {e}")
        return {"n_states": None, "n_events": None, "min_c": None, "max_c": None, "avg_c": None}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--platform", choices=["avr", "esp32"], required=True)
    parser.add_argument("--port", default=None)
    parser.add_argument("--manifest", default="output/sweep_manifest.csv")
    parser.add_argument("--ini-template", default=None,
                         help="default: output/platformio_sweep_<platform>.ini")
    parser.add_argument("--out", default=None, help="default: output/sweep_results_<platform>.csv")
    args = parser.parse_args()

    ini_template = args.ini_template or f"output/platformio_sweep_{args.platform}.ini"
    out_csv = args.out or f"output/sweep_results_{args.platform}.csv"

    port = find_port(args.port)
    if not port:
        print("GRESKA: nije pronadjen serijski port. Povezi plocu ili navedi --port.")
        return

    manifest = [m for m in load_manifest(args.manifest) if m["platform"] == args.platform]
    print(f"Platforma: {args.platform} | Port: {port} | Ukupno env-ova: {len(manifest)}")

    long_rows = []  # platform, pattern, param, param_value, metric, value
    n_states_mismatches = []  # sanity-check log: manifest value vs runtime-prijavljen N_STATES

    for m in manifest:
        env = m["env"]
        pattern = m["pattern"]
        param = m["param"]
        value = m["value"]

        print(f"\n--- {env} ({param}={value}) ---")
        set_active_env(ini_template, env)
        time.sleep(0.3)

        flash, ram = build_and_upload(env)
        if flash is None:
            print(f"Preskacem {env} zbog greske pri build-u/parsiranju Flash/RAM.")
            continue

        time.sleep(1.0)
        bench = read_benchmark_serial(port)

        print(f"-> Flash={flash}B RAM={ram}B Min={bench['min_c']} Avg={bench['avg_c']} Max={bench['max_c']}")

        # Sanity-check: da li firmware na plocici stvarno prijavljuje isti N_STATES
        # kao sto manifest ocekuje za ovaj env (hvata slucaj da je upload otisao
        # na pogresnu plocicu/env zbog npr. keširanog builda).
        if param == "n_states" and bench["n_states"] is not None:
            if bench["n_states"] != int(value):
                msg = f"{env}: manifest n_states={value}, firmware prijavljuje N_STATES={bench['n_states']}"
                print(f"-> UPOZORENJE (mismatch): {msg}")
                n_states_mismatches.append(msg)

        base = {"platform": args.platform, "pattern": pattern, "param": param, "param_value": value}
        long_rows.append({**base, "metric": "flash_bytes", "value": flash})
        long_rows.append({**base, "metric": "ram_bytes", "value": ram})
        if bench["min_c"] is not None:
            long_rows.append({**base, "metric": "min_cycles", "value": bench["min_c"]})
            long_rows.append({**base, "metric": "avg_cycles", "value": bench["avg_c"]})
            long_rows.append({**base, "metric": "max_cycles", "value": bench["max_c"]})

        time.sleep(1.0)

    if long_rows:
        os.makedirs(os.path.dirname(out_csv), exist_ok=True)
        with open(out_csv, "w", newline="") as f:
            fieldnames = ["platform", "pattern", "param", "param_value", "metric", "value"]
            w = csv.DictWriter(f, fieldnames=fieldnames)
            w.writeheader()
            w.writerows(long_rows)
        print(f"\nSacuvano {len(long_rows)} redova u {out_csv}")
        if n_states_mismatches:
            print(f"\nUPOZORENJE: {len(n_states_mismatches)} env-ova je prijavilo N_STATES razlicit od manifest vrednosti:")
            for msg in n_states_mismatches:
                print(f"  - {msg}")
        print("Sledeci korak: python curve_analysis.py --csv " + out_csv + " --metric avg_cycles")


if __name__ == "__main__":
    main()
