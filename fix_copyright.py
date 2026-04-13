import os

commands_dir = r"c:\Users\drewk\Documents\GitHub\FRC7531_2026\src\main\java\frc\robot\commands"

count = 0
for root, dirs, files in os.walk(commands_dir):
    for file in files:
        if file.endswith("Command.java"):
            filepath = os.path.join(root, file)
            
            with open(filepath, 'r', encoding='utf-8') as f:
                lines = f.readlines()
            
            # Fix first line if it has single / instead of //
            if lines and lines[0].strip().startswith("/ Copyright"):
                lines[0] = "// Copyright (c) FIRST and other WPILib contributors.\n"
                
                with open(filepath, 'w', encoding='utf-8') as f:
                    f.writelines(lines)
                
                print(f"[FIXED] {file}")
                count += 1
            elif lines and lines[0].strip().startswith("// Copyright"):
                print(f"[OK] {file}")
            else:
                print(f"[SKIP] {file} - unexpected first line")

print(f"\nFixed {count} files!")
