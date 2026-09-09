import subprocess
import time
import re
import csv
import serial
import serial.tools.list_ports


# ==============================================================================
# KONFIGURACIJA
# ==============================================================================
SERIAL_PORT = None  # Sam pronalazi COM3; ili upisi 'COM3'
BAUD_RATE = 9600
OUTPUT_CSV = "fsm_benchmark_results.csv"


# 12 FSM modela
FSM_ENVS = [
    "fsm_nested_switch_return",
    "fsm_nested_switch_break",
    "fsm_array_of_structs",
    "fsm_indexed_table",
    "fsm_state_object",
    "fsm_hsm_flat",
    "fsm_hsm_nested",
    "fsm_time_independent_timer0_isr",
    "fsm_array_of_structs_handler",
    "fsm_function_pointers",
    "fsm_santic_lookup_table"
]


OPTIMIZATIONS = ["-O0", "-Os", "-O2"]


def find_arduino_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = p.description.lower()
        if "arduino" in desc or "ch340" in desc or "usb serial" in desc or "ftdi" in desc or "com3" in p.device.lower():
            return p.device
    if ports:
        return ports[0].device
    return "COM3"


def update_platformio_ini(env_name, opt_flag):
    # Pravilno mapiranje flegova za svaki nivo
    if opt_flag == "-O0":
        unflags = "build_unflags = -Os -flto"
        flags = "build_flags = -O0 -fno-lto"
    elif opt_flag == "-Os":
        unflags = "" # Ne ponistavamo nista, zelimo standardni -Os -flto
        flags = "build_flags = -Os -flto"
    elif opt_flag == "-O2":
        unflags = "build_unflags = -Os"
        flags = "build_flags = -O2 -flto"


    content = f"""[platformio]
default_envs = {env_name}


[env]
platform = atmelavr
board = uno
framework = arduino
monitor_speed = 9600
{unflags}
{flags}


[env:fsm_nested_switch_return]
build_src_filter = -<*> +<fsm_nested_switch_return.cpp>


[env:fsm_nested_switch_break]
build_src_filter = -<*> +<fsm_nested_switch_break.cpp>


[env:fsm_array_of_structs]
build_src_filter = -<*> +<fsm_array_of_structs.cpp>


[env:fsm_indexed_table]
build_src_filter = -<*> +<fsm_indexed_table.cpp>


[env:fsm_state_object]
build_src_filter = -<*> +<fsm_state_object.cpp>


[env:fsm_hsm_flat]
build_src_filter = -<*> +<fsm_hsm_flat.cpp>


[env:fsm_hsm_nested]
build_src_filter = -<*> +<fsm_hsm_nested.cpp>


[env:fsm_time_independent_timer0_isr]
build_src_filter = -<*> +<fsm_time_independent_timer0_isr.cpp>


[env:fsm_santic_lookup_table]
build_src_filter = -<*> +<fsm_santic_lookup_table.cpp>
; ==============================================================================
; Varijante vernije akademskoj literaturi (vidi citat na vrhu svakog fajla)
; ==============================================================================
[env:fsm_array_of_structs_handler]
build_src_filter = -<*> +<fsm_array_of_structs_handler.cpp>


[env:fsm_function_pointers]
build_src_filter = -<*> +<fsm_function_pointers.cpp>


[env:fsm_hsm_statechart]
build_src_filter = -<*> +<fsm_hsm_statechart.cpp>
"""
    with open("platformio.ini", "w", encoding="utf-8") as f:
        f.write(content)


def build_and_upload(env_name, opt_flag):
    print(f"\n========================================================")
    print(f"Pokrecem: {env_name} sa flegom {opt_flag}...")
    print(f"========================================================")

    update_platformio_ini(env_name, opt_flag)
    time.sleep(0.3)

    # 1. Force clean - brisanje celog .pio foldera da se izbegne keširanje
    #    objektnih fajlova iz prethodnog environment-a (multiple definition bug)
    subprocess.run("if exist .pio rmdir /s /q .pio", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, shell=True)

    # 2. Upload preko powershell-a
    res = subprocess.run(f"pio run -e {env_name} -t upload", stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, shell=True)
    out = res.stdout

    if res.returncode != 0:
        print("-> GRESKA pri kompajliranju / uploada-u:")
        print(out[-800:])
        return None, None

    flash_bytes, ram_bytes = None, None

    # Ekstrakcija Flash i RAM memorije iz izlaza
    flash_match = re.search(r"used (\d+) bytes from 32256 bytes", out)
    if not flash_match:
        flash_match = re.search(r"Program:\s+(\d+)\s+bytes", out)
    if flash_match:
        flash_bytes = int(flash_match.group(1))

    ram_match = re.search(r"used (\d+) bytes from 2048 bytes", out)
    if not ram_match:
        ram_match = re.search(r"Data:\s+(\d+)\s+bytes", out)
    if ram_match:
        ram_bytes = int(ram_match.group(1))

    return flash_bytes, ram_bytes


def read_benchmark_serial(port):
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=6)
        time.sleep(2.0)  # Ceka restart Arduina

        ser.reset_input_buffer()
        ser.write(b'b')
        ser.flush()

        min_c, max_c, avg_c = None, None, None
        start_t = time.time()

        while time.time() - start_t < 6:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            if "Min cycles:" in line:
                m = re.search(r"Min cycles:\s*(\d+)", line)
                if m: min_c = int(m.group(1))
            elif "Max cycles:" in line:
                m = re.search(r"Max cycles:\s*(\d+)", line)
                if m: max_c = int(m.group(1))
            elif "Avg cycles:" in line:
                m = re.search(r"Avg cycles:\s*(\d+)", line)
                if m: avg_c = int(m.group(1))

            if min_c is not None and max_c is not None and avg_c is not None:
                break

        ser.close()
        return min_c, max_c, avg_c
    except Exception as e:
        print(f"-> GRESKA na serijskom portu: {e}")
        return None, None, None


def main():
    port = SERIAL_PORT or find_arduino_port()
    if not port:
        print("GRESKA: Arduino port nije pronadjen. Povezi plocu i probaj ponovo.")
        return

    print(f"Povezan Arduino na portu: {port}")
    print(f"Ukupno merenja: {len(FSM_ENVS)} modela x {len(OPTIMIZATIONS)} optimizacije = {len(FSM_ENVS) * len(OPTIMIZATIONS)} testova")

    results = []

    for env in FSM_ENVS:
        for opt in OPTIMIZATIONS:
            flash, ram = build_and_upload(env, opt)
            if flash is None:
                print(f"Preskacem merenje za {env} [{opt}] zbog greske.")
                continue

            time.sleep(1.0)
            min_c, max_c, avg_c = read_benchmark_serial(port)

            print(f"-> ZABELEZENO: Flash={flash} B | RAM={ram} B | Min={min_c} | Max={max_c} | Avg={avg_c} ciklusa")

            results.append({
                "FSM Pattern": env.replace("fsm_", ""),
                "Optimization": opt,
                "Flash (Bytes)": flash,
                "SRAM (Bytes)": ram,
                "Min Cycles": min_c,
                "Max Cycles": max_c,
                "Avg Cycles": avg_c
            })

            time.sleep(1.0)

    if results:
        fieldnames = ["FSM Pattern", "Optimization", "Flash (Bytes)", "SRAM (Bytes)", "Min Cycles", "Max Cycles", "Avg Cycles"]
        with open(OUTPUT_CSV, mode="w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(results)

        print(f"\n========================================================")
        print(f"SVA MERENJA ZAVRSENA USPESNO!")
        print(f"Tabela sacuvana u: {OUTPUT_CSV}")
        print(f"========================================================")


if __name__ == "__main__":
    main()