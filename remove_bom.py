import os
import sys

commands_dir = r"c:\Users\drewk\Documents\GitHub\FRC7531_2026\src\main\java\frc\robot\commands"

count = 0
for root, dirs, files in os.walk(commands_dir):
    for file in files:
        if file.endswith("Command.java"):
            filepath = os.path.join(root, file)
            
            # Read file as binary to preserve encoding issues
            with open(filepath, 'rb') as f:
                content = f.read()
            
            # Check for UTF-8 BOM (EF BB BF)
            if content.startswith(b'\xef\xbb\xbf'):
                # Remove BOM
                content = content[3:]
                
                # Write back without BOM
                with open(filepath, 'wb') as f:
                    f.write(content)
                
                print(f"[FIXED BOM] {file}")
                count += 1
            else:
                print(f"[OK] {file}")

print(f"\nFixed {count} files with BOM issues!")
print("All done! Now run: gradlew build")
