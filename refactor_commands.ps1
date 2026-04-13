# FRC7531 Command Class Standardization Refactoring Script
# This script renames all command files to PascalCaseCommand format and updates references

$commandsPath = "c:\Users\drewk\Documents\GitHub\FRC7531_2026\src\main\java\frc\robot\commands"
$robotContainerPath = "c:\Users\drewk\Documents\GitHub\FRC7531_2026\src\main\java\frc\robot\RobotContainer.java"

# Define all renaming mappings
$renameMap = @{
    'aimTurretToTarget.java' = 'AimTurretToTargetCommand.java'
    'alignTower.java' = 'AlignTowerCommand.java'
    'autoAimTurretToTarget.java' = 'AutoAimTurretToTargetCommand.java'
    'AutoHardcodeDepot_cmd.java' = 'AutoHardcodeDepotCommand.java'
    'AutoHardcodeDepotClose_cmd.java' = 'AutoHardcodeDepotCloseCommand.java'
    'AutoHardcodedScore_cmd.java' = 'AutoHardcodedScoreCommand.java'
    'AutoHardcodeHuman_cmd.java' = 'AutoHardcodeHumanCommand.java'
    'AutoIntake_cmd.java' = 'AutoIntakeCommand.java'
    'AutoIntakeOff_cmd.java' = 'AutoIntakeOffCommand.java'
    'AutoRev_cmd.java' = 'AutoRevCommand.java'
    'AutoShoot_cmd.java' = 'AutoShootCommand.java'
    'AutoShootOff_cmd.java' = 'AutoShootOffCommand.java'
    'AutoThroat_cmd.java' = 'AutoThroatCommand.java'
    'AutoThroatHang_cmd.java' = 'AutoThroatHangCommand.java'
    'AutoThroatHuman_cmd.java' = 'AutoThroatHumanCommand.java'
    'AutoThroatOff_cmd.java' = 'AutoThroatOffCommand.java'
    'fireShooter.java' = 'FireShooterCommand.java'
    'foldIntake_cmd.java' = 'FoldIntakeCommand.java'
    'HangerDefault_cmd.java' = 'HangerDefaultCommand.java'
    'HangLevel1_cmd.java' = 'HangLevel1Command.java'
    'HangLevel2_cmd.java' = 'HangLevel2Command.java'
    'HangLevel3_cmd.java' = 'HangLevel3Command.java'
    'HangReturn_cmd.java' = 'HangReturnCommand.java'
    'HangReturnManual_cmd.java' = 'HangReturnManualCommand.java'
    'HangUpManual_cmd.java' = 'HangUpManualCommand.java'
    'hopperDefault_cmd.java' = 'HopperDefaultCommand.java'
    'intakeToggle_cmd.java' = 'IntakeToggleCommand.java'
    'joystickInversion.java' = 'JoystickInversionCommand.java'
    'lobShooter.java' = 'LobShooterCommand.java'
    'lowerHood.java' = 'LowerHoodCommand.java'
    'manualFoldIntake_cmd.java' = 'ManualFoldIntakeCommand.java'
    'manualHood_cmd.java' = 'ManualHoodCommand.java'
    'manualShooter.java' = 'ManualShooterCommand.java'
    'manualTurret.java' = 'ManualTurretCommand.java'
    'manualUnfoldIntake_cmd.java' = 'ManualUnfoldIntakeCommand.java'
    'outakeToggle_cmd.java' = 'OutakeToggleCommand.java'
    'rollersForwardManual_cmd.java' = 'RollersForwardManualCommand.java'
    'rollersOff_cmd.java' = 'RollersOffCommand.java'
    'rollersOn_cmd.java' = 'RollersOnCommand.java'
    'rollersReverseManual_cmd.java' = 'RollersReverseManualCommand.java'
    'setTurretAngle_cmd.java' = 'SetTurretAngleCommand.java'
    'startThroat.java' = 'StartThroatCommand.java'
    'stopThroat.java' = 'StopThroatCommand.java'
    'stopTurret.java' = 'StopTurretCommand.java'
    'unfoldIntake_cmd.java' = 'UnfoldIntakeCommand.java'
}

Write-Host "FRC7531 Command Standardization Refactoring" -ForegroundColor Cyan -BackgroundColor Black
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host ""

# Function to extract old class name from new filename
function Get-OldClassName {
    param($oldFilename)
    return $oldFilename -replace '\.java$'
}

function Get-NewClassName {
    param($newFilename)
    return $newFilename -replace '\.java$'
}

Write-Host "Step 1: Renaming files and updating class declarations..." -ForegroundColor Yellow

$fileCount = 0
foreach ($oldFile in $renameMap.Keys) {
    $newFile = $renameMap[$oldFile]
    
    # Find all subdirectories for this file
    $foundFiles = Get-ChildItem -Path $commandsPath -Recurse -Filter $oldFile
    
    foreach ($file in $foundFiles) {
        $oldClassName = Get-OldClassName $oldFile
        $newClassName = Get-NewClassName $newFile
        
        # Read file content
        $content = Get-Content -Path $file.FullName -Raw -Encoding UTF8
        
        # Replace class declaration
        $content = $content -replace "public class $oldClassName", "public class $newClassName"
        $content = $content -replace "class $oldClassName", "class $newClassName"
        
        # Write to new file
        $newFilePath = Join-Path $file.Directory.FullName $newFile
        Set-Content -Path $newFilePath -Value $content -Encoding UTF8
        
        # Remove old file
        Remove-Item -Path $file.FullName -Force
        
        $fileCount++
        Write-Host "  [RENAMED] $oldFile -> $newFile" -ForegroundColor Green
    }
}

Write-Host ""
Write-Host "Step 1 Complete: $fileCount files renamed" -ForegroundColor Green
Write-Host ""
Write-Host "NEXT STEPS:" -ForegroundColor Cyan
Write-Host "1. VS Code will auto-reload the renamed files"
Write-Host "2. Run: gradle build (to see RobotContainer import errors)"
Write-Host "3. Use VS Code Find and Replace to update imports in RobotContainer.java"
Write-Host ""
