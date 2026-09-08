import subprocess
import time
import re
import csv
import serial
import serial.tools.list_ports

# ==============================================================================
# KONFIGURACIJA
# ==============================================================================
SERIAL_PORT = None  # Sam pronalazi port (CP210x/CH340/FTDI); ili upisi npr. 'COM4'
BAUD_RATE = 115200
OUTPUT_CSV = "fsm_benchmark_results_esp32.csv"

# 8 FSM modela (isti patterni kao u ARDUINO_SFSM, adaptirani za ESP32)
FSM_ENVS = [
    "fsm_empty",
    "fsm_nested_switch_return",
    "fsm_nested_switch_break",
    "fsm_array_of_structs",
    "fsm_indexed_table",
    "fsm_state_object",
    "fsm_hsm_flat",
    "fsm_hsm_nested",
    "fsm_time_independent_timer_isr",  # napomena: 'b' ovde pokrece Scenario B, ne 1000-tranzicija benchmark
    "fsm_lookup_table_santic"
]

# Bez -flto na ESP32: precompiled SDK biblioteke (WiFi/BT/...) nisu gradjene sa LTO,
# pa bi njegovo ukljucivanje moglo da slomi linkovanje (za razliku od avr-gcc gde je ceo build iz izvora).
OPTIMIZATIONS = ["-O0", "-Os", "-O2"]


def find_esp32_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = p.description.lower()
        if "cp210" in desc or "ch340" in desc or "silicon labs" in desc or "usb serial" in desc or "usb-serial" in desc or "ftdi" in desc:
            return p.device
    if ports:
        return ports[0].device
    return "COM4"


def update_platformio_ini(env_name, opt_flag):
    if opt_flag == "-O0":
        unflags = "build_unflags = -Os"
        flags = "build_flags = -O0"
    elif opt_flag == "-Os":
        unflags = "build_unflags ="
        flags = "build_flags = -Os"
    elif opt_flag == "-O2":
        unflags = "build_unflags = -Os"
        flags = "build_flags = -O2"

    content = f"""[platformio]
default_envs = {env_name}

[env]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
{unflags}
{flags}

[env:fsm_empty]
build_src_filter = -<*> +<fsm_empty.cpp>

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

[env:fsm_time_independent_timer_isr]
build_src_filter = -<*> +<fsm_time_independent_timer_isr.cpp>

[env:fsm_lookup_table_santic]
build_src_filter = -<*> +<fsm_lookup_table_santic.cpp>

"""
    with open("platformio.ini", "w", encoding="utf-8") as f:
        f.write(content)


def build_and_upload(env_name, opt_flag):
    print(f"\n========================================================")
    print(f"Pokrecem: {env_name} sa flegom {opt_flag}...")
    print(f"========================================================")

    update_platformio_ini(env_name, opt_flag)
    time.sleep(0.3)

    subprocess.run("pio run -t clean", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, shell=True)

    res = subprocess.run(f"pio run -e {env_name} -t upload", stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, shell=True)
    out = res.stdout

    if res.returncode != 0:
        print("-> GRESKA pri kompajliranju / uploada-u:")
        print(out[-800:])
        return None, None

    flash_bytes, ram_bytes = None, None

    # PlatformIO za ESP32 stampa "RAM:   [..] X% (used NNNN bytes from MMMM bytes)"
    # i "Flash: [..] X% (used NNNN bytes from MMMM bytes)" - svaka na svojoj liniji.
    for line in out.splitlines():
        if "RAM:" in line and ram_bytes is None:
            m = re.search(r"used (\d+) bytes", line)
            if m:
                ram_bytes = int(m.group(1))
        elif "Flash:" in line and flash_bytes is None:
            m = re.search(r"used (\d+) bytes", line)
            if m:
                flash_bytes = int(m.group(1))

    return flash_bytes, ram_bytes


def read_benchmark_serial(port):
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=6)
        time.sleep(3.0)  # ESP32 boot (bootloader + Arduino init) traje duze nego AVR reset

        ser.reset_input_buffer()
        ser.write(b'b')
        ser.flush()

        min_c, max_c, avg_c = None, None, None
        free_heap, min_free_heap, stack_hwm = None, None, None
        start_t = time.time()

        while time.time() - start_t < 8:
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
            elif "Free heap:" in line:
                m = re.search(r"Free heap:\s*(\d+)", line)
                if m: free_heap = int(m.group(1))
            elif "Min free heap:" in line:
                m = re.search(r"Min free heap:\s*(\d+)", line)
                if m: min_free_heap = int(m.group(1))
            elif "Stack HWM:" in line:
                m = re.search(r"Stack HWM:\s*(\d+)", line)
                if m: stack_hwm = int(m.group(1))

            if min_c is not None and max_c is not None and avg_c is not None and stack_hwm is not None:
                break

        ser.close()
        return min_c, max_c, avg_c, free_heap, min_free_heap, stack_hwm
    except Exception as e:
        print(f"-> GRESKA na serijskom portu: {e}")
        return None, None, None, None, None, None


def main():
    port = SERIAL_PORT or find_esp32_port()
    if not port:
        print("GRESKA: ESP32 port nije pronadjen. Povezi plocu i probaj ponovo.")
        return

    print(f"Povezan ESP32 na portu: {port}")
    print(f"Ukupno merenja: {len(FSM_ENVS)} modela x {len(OPTIMIZATIONS)} optimizacije = {len(FSM_ENVS) * len(OPTIMIZATIONS)} testova")

    results = []

    for env in FSM_ENVS:
        for opt in OPTIMIZATIONS:
            flash, ram = build_and_upload(env, opt)
            if flash is None:
                print(f"Preskacem merenje za {env} [{opt}] zbog greske.")
                continue

            time.sleep(1.0)
            min_c, max_c, avg_c, free_heap, min_free_heap, stack_hwm = read_benchmark_serial(port)

            print(f"-> ZABELEZENO: Flash={flash} B | RAM={ram} B | Min={min_c} | Max={max_c} | Avg={avg_c} ciklusa | "
                  f"FreeHeap={free_heap} | MinFreeHeap={min_free_heap} | StackHWM={stack_hwm}")

            results.append({
                "FSM Pattern": env.replace("fsm_", ""),
                "Optimization": opt,
                "Flash (Bytes)": flash,
                "RAM (Bytes)": ram,
                "Min Cycles": min_c,
                "Max Cycles": max_c,
                "Avg Cycles": avg_c,
                "Free Heap (Bytes)": free_heap,
                "Min Free Heap (Bytes)": min_free_heap,
                "Stack HWM (Bytes)": stack_hwm,
            })

            time.sleep(1.0)

    if results:
        fieldnames = ["FSM Pattern", "Optimization", "Flash (Bytes)", "RAM (Bytes)",
                      "Min Cycles", "Max Cycles", "Avg Cycles",
                      "Free Heap (Bytes)", "Min Free Heap (Bytes)", "Stack HWM (Bytes)"]
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
