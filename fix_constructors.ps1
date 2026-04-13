# Fix all constructor method names to match new class names
# This script finds all command class files and updates their constructor names

$commandsPath = "c:\Users\drewk\Documents\GitHub\FRC7531_2026\src\main\java\frc\robot\commands"

$renameMap = @{
    'AimTurretToTargetCommand.java' = @{ old = 'aimTurretToTarget'; new = 'AimTurretToTargetCommand' }
    'AlignTowerCommand.java' = @{ old = 'alignTower'; new = 'AlignTowerCommand' }
    'AutoAimTurretToTargetCommand.java' = @{ old = 'autoAimTurretToTarget'; new = 'AutoAimTurretToTargetCommand' }
    'AutoHardcodeDepotCommand.java' = @{ old = 'AutoHardcodeDepot_cmd'; new = 'AutoHardcodeDepotCommand' }
    'AutoHardcodeDepotCloseCommand.java' = @{ old = 'AutoHardcodeDepotClose_cmd'; new = 'AutoHardcodeDepotCloseCommand' }
    'AutoHardcodedScoreCommand.java' = @{ old = 'AutoHardcodedScore_cmd'; new = 'AutoHardcodedScoreCommand' }
    'AutoHardcodeHumanCommand.java' = @{ old = 'AutoHardcodeHuman_cmd'; new = 'AutoHardcodeHumanCommand' }
    'AutoIntakeCommand.java' = @{ old = 'AutoIntake_cmd'; new = 'AutoIntakeCommand' }
    'AutoIntakeOffCommand.java' = @{ old = 'AutoIntakeOff_cmd'; new = 'AutoIntakeOffCommand' }
    'AutoRevCommand.java' = @{ old = 'AutoRev_cmd'; new = 'AutoRevCommand' }
    'AutoShootCommand.java' = @{ old = 'AutoShoot_cmd'; new = 'AutoShootCommand' }
    'AutoShootOffCommand.java' = @{ old = 'AutoShootOff_cmd'; new = 'AutoShootOffCommand' }
    'AutoThroatCommand.java' = @{ old = 'AutoThroat_cmd'; new = 'AutoThroatCommand' }
    'AutoThroatHangCommand.java' = @{ old = 'AutoThroatHang_cmd'; new = 'AutoThroatHangCommand' }
    'AutoThroatHumanCommand.java' = @{ old = 'AutoThroatHuman_cmd'; new = 'AutoThroatHumanCommand' }
    'AutoThroatOffCommand.java' = @{ old = 'AutoThroatOff_cmd'; new = 'AutoThroatOffCommand' }
    'FireShooterCommand.java' = @{ old = 'fireShooter'; new = 'FireShooterCommand' }
    'FoldIntakeCommand.java' = @{ old = 'foldIntake_cmd'; new = 'FoldIntakeCommand' }
    'HangerDefaultCommand.java' = @{ old = 'HangerDefault_cmd'; new = 'HangerDefaultCommand' }
    'HangLevel1Command.java' = @{ old = 'HangLevel1_cmd'; new = 'HangLevel1Command' }
    'HangLevel2Command.java' = @{ old = 'HangLevel2_cmd'; new = 'HangLevel2Command' }
    'HangLevel3Command.java' = @{ old = 'HangLevel3_cmd'; new = 'HangLevel3Command' }
    'HangReturnCommand.java' = @{ old = 'HangReturn_cmd'; new = 'HangReturnCommand' }
    'HangReturnManualCommand.java' = @{ old = 'HangReturnManual_cmd'; new = 'HangReturnManualCommand' }
    'HangUpManualCommand.java' = @{ old = 'HangUpManual_cmd'; new = 'HangUpManualCommand' }
    'HopperDefaultCommand.java' = @{ old = 'hopperDefault_cmd'; new = 'HopperDefaultCommand' }
    'IntakeToggleCommand.java' = @{ old = 'intakeToggle_cmd'; new = 'IntakeToggleCommand' }
    'JoystickInversionCommand.java' = @{ old = 'joystickInversion'; new = 'JoystickInversionCommand' }
    'LobShooterCommand.java' = @{ old = 'lobShooter'; new = 'LobShooterCommand' }
    'LowerHoodCommand.java' = @{ old = 'lowerHood'; new = 'LowerHoodCommand' }
    'ManualFoldIntakeCommand.java' = @{ old = 'manualFoldIntake_cmd'; new = 'ManualFoldIntakeCommand' }
    'ManualHoodCommand.java' = @{ old = 'manualHood_cmd'; new = 'ManualHoodCommand' }
    'ManualShooterCommand.java' = @{ old = 'manualShooter'; new = 'ManualShooterCommand' }
    'ManualTurretCommand.java' = @{ old = 'manualTurret'; new = 'ManualTurretCommand' }
    'ManualUnfoldIntakeCommand.java' = @{ old = 'manualUnfoldIntake_cmd'; new = 'ManualUnfoldIntakeCommand' }
    'OutakeToggleCommand.java' = @{ old = 'outakeToggle_cmd'; new = 'OutakeToggleCommand' }
    'RollersForwardManualCommand.java' = @{ old = 'rollersForwardManual_cmd'; new = 'RollersForwardManualCommand' }
    'RollersOffCommand.java' = @{ old = 'rollersOff_cmd'; new = 'RollersOffCommand' }
    'RollersOnCommand.java' = @{ old = 'rollersOn_cmd'; new = 'RollersOnCommand' }
    'RollersReverseManualCommand.java' = @{ old = 'rollersReverseManual_cmd'; new = 'RollersReverseManualCommand' }
    'SetTurretAngleCommand.java' = @{ old = 'setTurretAngle_cmd'; new = 'SetTurretAngleCommand' }
    'StartThroatCommand.java' = @{ old = 'startThroat'; new = 'StartThroatCommand' }
    'StopThroatCommand.java' = @{ old = 'stopThroat'; new = 'StopThroatCommand' }
    'StopTurretCommand.java' = @{ old = 'stopTurret'; new = 'StopTurretCommand' }
    'UnfoldIntakeCommand.java' = @{ old = 'unfoldIntake_cmd'; new = 'UnfoldIntakeCommand' }
}

Write-Host "Fixing constructor method names in all command files..." -ForegroundColor Cyan
$count = 0

foreach ($filename in $renameMap.Keys) {
    $oldName = $renameMap[$filename].old
    $newName = $renameMap[$filename].new
    
    $foundFiles = Get-ChildItem -Path $commandsPath -Recurse -Filter $filename
    
    foreach ($file in $foundFiles) {
        $content = Get-Content -Path $file.FullName -Raw -Encoding UTF8
        
        # Replace constructor method names
        # Match: public oldName( or public oldName (
        $content = $content -replace "public\s+$([regex]::Escape($oldName))\s*\(", "public $newName("
        
        Set-Content -Path $file.FullName -Value $content -Encoding UTF8
        $count++
        Write-Host "  [FIXED] $filename" -ForegroundColor Green
    }
}

Write-Host ""
Write-Host "Constructor method names fixed in $count files" -ForegroundColor Green
Write-Host ""
Write-Host "Now run: gradlew build" -ForegroundColor Cyan
