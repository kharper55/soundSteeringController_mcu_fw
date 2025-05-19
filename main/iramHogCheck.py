import re

map_file = "C:/Users/kharp/Desktop/ss_ctrl_fw_v2.0/build/soundSteering_controller_revB_fw1_00.map"

with open(map_file, "r", encoding="utf-8") as f:
    lines = f.readlines()

in_iram = False
iram_entries = []

for line in lines:
    if ".iram0.text" in line:
        in_iram = True
        continue
    if in_iram:
        if line.strip() == "":
            break
        match = re.match(r'\s*(0x[0-9a-f]+)\s+(0x[0-9a-f]+)\s+(.+)', line)
        if match:
            addr, size, source = match.groups()
            iram_entries.append((int(size, 16), source.strip()))

# Sort by size descending
iram_entries.sort(reverse=True)

print("Top IRAM users:")
for size, source in iram_entries[:20]:
    print(f"{size:6} bytes - {source}")
