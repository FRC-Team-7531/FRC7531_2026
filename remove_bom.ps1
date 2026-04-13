# Remove UTF-8 BOM from command files
$commandsDir = "c:\Users\drewk\Documents\GitHub\FRC7531_2026\src\main\java\frc\robot\commands"

$files = Get-ChildItem -Path $commandsDir -Recurse -Filter "*.java" | Where-Object { $_.Name -like "*Command.java" }

$bom = [char]0xFEFF
$count = 0

foreach ($file in $files) {
    # Read with specific encoding
    $content = [System.IO.File]::ReadAllText($file.FullName)
    
    # Check if file has BOM
    if ($content.StartsWith($bom)) {
        # Remove the BOM character
        $cleanContent = $content.Substring(1)
        
        # Write back without BOM
        [System.IO.File]::WriteAllText($file.FullName, $cleanContent, [System.Text.Encoding]::UTF8)
        
        Write-Host "[FIXED BOM] $($file.Name)" -ForegroundColor Green
        $count++
    } else {
        Write-Host "[OK] $($file.Name)" -ForegroundColor Gray
    }
}

Write-Host "`nFixed $count files with BOM issues!" -ForegroundColor Cyan
Write-Host "All done! Now run: gradlew build" -ForegroundColor Yellow
