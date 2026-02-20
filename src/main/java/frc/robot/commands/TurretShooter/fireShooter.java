// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretShooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SS_Drivetrain;
import frc.robot.subsystems.SS_Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class fireShooter extends Command {
  public SS_Shooter shooter;
  public SS_Drivetrain drivetrain;

  public double distance;
  public double hoodAngle;
  public Translation2d botPose;
  public Translation2d targetPose;
  public double targetAngle;

  /** Creates a new startShooter. */
  public fireShooter(SS_Shooter ss_shooter, SS_Drivetrain ss_drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ss_shooter);
    this.shooter = ss_shooter;
    this.drivetrain = ss_drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPose = drivetrain.targetPose;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (targetPose != drivetrain.targetPose) {
      targetPose = drivetrain.targetPose;
    }
    
    botPose = drivetrain.poseEstimator.getEstimatedPosition().getTranslation();
    distance = botPose.getDistance(targetPose);
    targetAngle = shooter.calculateGoalAngle(distance);
    shooter.setHoodAngle(targetAngle);
    hoodAngle = shooter.leftHoodLifter.getPosition();
    shooter.setVelocity(distance, hoodAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
