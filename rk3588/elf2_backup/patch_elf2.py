#!/usr/bin/env python3
"""Fix ELF2 code protocol issues"""
import shutil

# === 1. Fix trash_detector.py ===
shutil.copy('/home/elf/trash_detector.py', '/home/elf/trash_detector.py.bak2')
with open('/home/elf/trash_detector.py', 'r') as f:
    content = f.read()

# Fix BinaryFrame.pack(): XOR->SUM checksum, add type+len fields
old_pack = '''    def pack(self) -> bytes:
        flags = (self.category & 0x01) | ((self.valid & 0x01) << 1)
        payload  = struct.pack('<B',  flags)            # [2]
        payload += struct.pack('<H',  self.u)           # [3-4]
        payload += struct.pack('<H',  self.v)           # [5-6]
        payload += struct.pack('<h',  self.dx)          # [7-8]
        payload += struct.pack('<h',  self.dy)          # [9-10]
        payload += struct.pack('<h',  self.x_mm)        # [11-12]
        payload += struct.pack('<h',  self.y_mm)        # [13-14]
        payload += struct.pack('<H',  self.arrive_ms)   # [15-16]
        checksum = 0
        for b in payload:
            checksum ^= b
        payload += struct.pack('<B', checksum)           # [17]
        return self.HEADER + payload'''

new_pack = '''    def pack(self) -> bytes:
        """STM32 protocol: AA 55 type len payload SUM-checksum"""
        flags = (self.category & 0x01) | ((self.valid & 0x01) << 1)
        payload  = struct.pack('<B',  flags)
        payload += struct.pack('<H',  self.u)
        payload += struct.pack('<H',  self.v)
        payload += struct.pack('<h',  self.dx)
        payload += struct.pack('<h',  self.dy)
        payload += struct.pack('<h',  self.x_mm)
        payload += struct.pack('<h',  self.y_mm)
        payload += struct.pack('<H',  self.arrive_ms)
        cmd_type = 0x06
        length = len(payload)
        checksum = (cmd_type + length) & 0xFF
        for b in payload:
            checksum = (checksum + b) & 0xFF
        return self.HEADER + bytes([cmd_type, length]) + payload + bytes([checksum])'''

if old_pack in content:
    content = content.replace(old_pack, new_pack)
    print('[OK] trash_detector: BinaryFrame.pack() fixed')
else:
    print('[FAIL] trash_detector: pack() not found')
    import sys; sys.exit(1)

with open('/home/elf/trash_detector.py', 'w') as f:
    f.write(content)

# === 2. Fix motor_controller.py camera_ns ===
shutil.copy('/home/elf/motor_controller.py', '/home/elf/motor_controller.py.bak')
with open('/home/elf/motor_controller.py', 'r') as f:
    mc = f.read()

old_ns = 'ascamera_hp60cn'
new_ns = 'ascamera_hp60c'
if old_ns in mc:
    mc = mc.replace(old_ns, new_ns)
    print('[OK] motor_controller: camera_ns fixed')
else:
    print('[WARN] motor_controller: old camera_ns not found, checking...')
    if 'hp60c' in mc:
        print('[OK] motor_controller: already has hp60c')

with open('/home/elf/motor_controller.py', 'w') as f:
    f.write(mc)

# === 3. Fix .bashrc ===
bashrc_path = '/home/elf/.bashrc'
shutil.copy(bashrc_path, bashrc_path + '.bak')
with open(bashrc_path, 'r') as f:
    lines = f.readlines()

new_lines = []
for line in lines:
    if 'source /opt/ros/noetic/setup.bash' in line:
        print('[OK] .bashrc: removed dead ROS1 noetic line')
        continue
    if 'source /opt/ros/humble/setup.bash' in line:
        if new_lines and 'source /opt/ros/humble/setup.bash' in new_lines[-1]:
            print('[OK] .bashrc: removed duplicate humble source')
            continue
    new_lines.append(line)

with open(bashrc_path, 'w') as f:
    f.writelines(new_lines)

print('\n=== All fixes applied successfully ===')
print('Backups: .bak .bak2 files in /home/elf/')
