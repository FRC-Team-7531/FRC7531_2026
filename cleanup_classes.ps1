# Comprehensive fix for command class names - fixes doubled names and encoding issues
# Maps old filename to the correct clean-up

$commandsPath = "c:\Users\drewk\Documents\GitHub\FRC7531_2026\src\main\java\frc\robot\commands"

# Define all commands with their correct class names
$commands = @(
    'AimTurretToTargetCommand',
    'AlignTowerCommand',
    'AutoAimTurretToTargetCommand',
    'AutoHardcodeDepotCommand',
    'AutoHardcodeDepotCloseCommand',
    'AutoHardcodedScoreCommand',
    'AutoHardcodeHumanCommand',
    'AutoIntakeCommand',
    'AutoIntakeOffCommand',
    'AutoRevCommand',
    'AutoShootCommand',
    'AutoShootOffCommand',
    'AutoThroatCommand',
    'AutoThroatHangCommand',
    'AutoThroatHumanCommand',
    'AutoThroatOffCommand',
    'FireShooterCommand',
    'FoldIntakeCommand',
    'HangerDefaultCommand',
    'HangLevel1Command',
    'HangLevel2Command',
    'HangLevel3Command',
    'HangReturnCommand',
    'HangReturnManualCommand',
    'HangUpManualCommand',
    'HopperDefaultCommand',
    'IntakeToggleCommand',
    'JoystickInversionCommand',
    'LobShooterCommand',
    'LowerHoodCommand',
    'ManualFoldIntakeCommand',
    'ManualHoodCommand',
    'ManualShooterCommand',
    'ManualTurretCommand',
    'ManualUnfoldIntakeCommand',
    'OutakeToggleCommand',
    'RollersForwardManualCommand',
    'RollersOffCommand',
    'RollersOnCommand',
    'RollersReverseManualCommand',
    'SetTurretAngleCommand',
    'StartThroatCommand',
    'StopThroatCommand',
    'StopTurretCommand',
    'UnfoldIntakeCommand'
)

Write-Host "Fixing all command class files..." -ForegroundColor Cyan

foreach ($className in $commands) {
    $foundFiles = Get-ChildItem -Path $commandsPath -Recurse -Filter "$className.java"
    
    foreach ($file in $foundFiles) {
        $content = Get-Content -Path $file.FullName -Raw -Encoding UTF8
        
        # Fix doubled class names (e.g., AlignTowerCommandCommand -> AlignTowerCommand)
        $content = $content -replace "public class $className$className", "public class $className"
        
        # Fix constructor with doubled name
        $doubledName = "$className$className"
        $content = $content -replace "public\s+$([regex]::Escape($doubledName))\s*\(", "public $className("
        
        # Ensure the public class line is correct
        if ($content -notmatch "public class $className extends Command") {
            $content = $content -replace "public class \w+Command extends Command", "public class $className extends Command"
        }
        
        Set-Content -Path $file.FullName -Value $content -Encoding UTF8
        Write-Host "  [FIXED] $className" -ForegroundColor Green
    }
}

Write-Host ""
Write-Host "All command files have been fixed!" -ForegroundColor Green
Write-Host ""
Write-Host "Now run: gradlew build" -ForegroundColor Cyan
