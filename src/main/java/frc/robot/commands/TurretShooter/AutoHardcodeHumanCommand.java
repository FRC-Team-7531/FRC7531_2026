// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretShooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SS_Drivetrain;
import frc.robot.subsystems.SS_Shooter;
import frc.robot.subsystems.SS_Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoHardcodeHumanCommand extends Command {
  //Hardcode for auto middle Human
  
  SS_Turret turret;
  SS_Shooter shooter;

  PIDController tController = new PIDController(8, 0.1, 0);
  double tSpeed;
  int counter;

  /** Creates a new AutoHardcodedScore_cmd. */
  public AutoHardcodeHumanCommand(SS_Turret ss_turret, SS_Shooter ss_shooter, SS_Drivetrain ss_drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ss_turret, ss_shooter);
    this.turret = ss_turret;
    this.shooter = ss_shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tController.setSetpoint(-0.148);
    tController.setTolerance(0.002);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tSpeed = tController.calculate(turret.getTurretRotation());
    if (tController.atSetpoint()) {
      turret.setRawSpeed(0);
    } else {
      turret.setRawSpeed(tSpeed);
    }
    
    shooter.setSpeed(0.9); // 6.476/9

    shooter.hoodLifter.setPosition(0.52);
    counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (tController.atSetpoint() && (shooter.hoodLifter.getPosition() == 0.52)) {// || counter > 2*1000 / 20.0) {
      turret.setRawSpeed(0);
      return true;
    } else{
      return false;
    }
  }
}



