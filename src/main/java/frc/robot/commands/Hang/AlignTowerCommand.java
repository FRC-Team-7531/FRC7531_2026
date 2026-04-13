// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hang;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SS_Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignTowerCommand extends Command {
  public RobotContainer robotContainer;
  public SS_Drivetrain drivetrain;
  SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
  NetworkTable limelightTableBarbuda = NetworkTableInstance.getDefault().getTable("limelight-barbuda");
  Translation2d estimatedPose;

  PIDController rController = new PIDController(0.008, 0.02, 0);
  PIDController xController = new PIDController(3.5, 6.5, 0);
  PIDController yController = new PIDController(3.5, 6.5, 0);

  Translation2d targetPose;
  Translation2d poseDifference;
  Double targetXPoseDifference;
  Double targetYPoseDifference;
  private double targetAngle;

  double currentAngle;

  double rSpeed;
  double xSpeed;
  double ySpeed;

  /** Creates a new alignTower. */
  public AlignTowerCommand(SS_Drivetrain ss_drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ss_drivetrain);
    this.drivetrain = ss_drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rController.setSetpoint(0);
    rController.setTolerance(0.2);

    xController.setSetpoint(0);
    xController.setTolerance(0.01);
    xController.setIZone(0.35);

    yController.setSetpoint(0);
    yController.setTolerance(0.01);
    yController.setIZone(0.35);

    targetPose = drivetrain.towerPose;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    estimatedPose = drivetrain.poseEstimator.getEstimatedPosition().getTranslation();
    poseDifference = targetPose.minus(estimatedPose);

    currentAngle = drivetrain.pidgey.getRotation2d().getDegrees();
    targetXPoseDifference = poseDifference.getMeasureX().in(Meters);
    targetYPoseDifference = poseDifference.getMeasureY().in(Meters);

    if (Math.floorMod((int) currentAngle, 360) < 180) {
      targetAngle = Math.IEEEremainder(currentAngle, 360);
    } else {
      targetAngle = Math.IEEEremainder(currentAngle, 360) - 360;
    }

    rSpeed = rController.calculate(targetAngle);
    xSpeed = -xController.calculate(targetXPoseDifference);
    ySpeed = -yController.calculate(targetYPoseDifference);

    if (rController.atSetpoint()) {
      rSpeed = 0;
    }
    if (xController.atSetpoint()) {
      xSpeed = 0;
    }
    if (yController.atSetpoint()) {
      ySpeed = 0;
    }

    SmartDashboard.putNumber("rSpeed", rSpeed);
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("ccccurrentAngle", currentAngle);
    SmartDashboard.putNumber("ttttargetAngle", targetAngle);

    drivetrain.setControl(drive.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(rSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (rController.atSetpoint() && xController.atSetpoint() && yController.atSetpoint()) {
      return true;
    } else {
      return false;
    }
  }
}



