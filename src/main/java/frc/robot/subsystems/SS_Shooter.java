// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SS_Shooter extends SubsystemBase {
  public final double shooterMaxSpeed = 9; //Theoretical, not gonna be exact yet

  // Constants stuff all in meters -ish
  public final double lipHeight = 1.8288;
  public final double hoopRadius = 0.5969;
  public final double gravity = 4.90335;
  public final double minimumClearance = 0.08;
  public final double nearBound = 1.50617974628;

  public final double shooterAxleX = 0.2413; // a_x
  public final double shooterAxleY = 0.0762; // a_y
  public final double actuatorMinimum = 0.1651; // e_min
  public final double hoodRadius = 0.15875; // h_r
  public final double maximumExtension = 0.1; // always 100 mm

  public final double hoodAngleBack = 0.174533;
  public final double shooterHeight = 0.3048;

  // Set these to change the desired trajectory
  public double targetHeight = 1.8 - shooterHeight; // h
  public double lipClearance = 0.5; // b
  public double horizontalClearance = 0.2;

  // Precalculated stuff
  public final double clearanceHeight = lipHeight + lipClearance;
  public final double passableHeight = lipHeight + minimumClearance;
  public final double nearCenterDistance = hoopRadius + horizontalClearance;

  // Stuff to be calculated
  public double lipDistance;
  public double targetVelocity;
  public double targetAngle;
  public double targetPosition;
  public double nearDistance;
  public double hoodAngle;

  // Hard goods
  public PWM hoodLifter = new PWM(9);
  public TalonFX leftShooter = new TalonFX(42, "rio");
  public TalonFX rightShooter = new TalonFX(41, "rio");

  // NetworkTable stuff
  public NetworkTableInstance inst = NetworkTableInstance.getDefault();
  public NetworkTableEntry actuatorPosition = inst.getTable("Shooter").getEntry("Actuator Position");
  public NetworkTableEntry shooterSpeed = inst.getTable("Shooter").getEntry("Shooter Speed");
  public NetworkTableEntry hoodEntry = inst.getTable("Shooter").getEntry("Hood Angle");

  /** Creates a new SS_Shooter. */
  public SS_Shooter() {
    actuatorPosition.setDouble(0.0);
    shooterSpeed.setDouble(0.0);
    leftShooter.setNeutralMode(NeutralModeValue.Coast);
    rightShooter.setNeutralMode(NeutralModeValue.Coast);
    hoodEntry.setDouble(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    actuatorPosition = inst.getTable("Shooter").getEntry("Actuator Position");
    shooterSpeed = inst.getTable("Shooter").getEntry("Shooter Speed");
    hoodEntry.setDouble(180 / Math.PI * hoodAngle);
  }

  public double calculateGoalAngle(double distance) {
    lipDistance = distance - hoopRadius;
    nearDistance = distance - hoopRadius - lipClearance;

    // if (distance < nearBound) {
    //  targetAngle = Math.atan((lipHeight*distance)/(nearDistance*nearCenterDistance)-(targetHeight*nearDistance)/(distance*nearCenterDistance));
    //} // else {
       targetAngle = Math.atan((distance*clearanceHeight)/(lipDistance*hoopRadius) - (targetHeight*lipDistance)/(distance*hoopRadius));
    // }

    return targetAngle;
  }

  public void setVelocity(double distance, double shooterPosition, double drivetrainVelocity) {
    lipDistance = distance - hoopRadius;
    nearDistance = distance - hoopRadius - lipClearance;
    hoodAngle = Math.PI/2 - 0.591377*shooterPosition - 0.380351 + hoodAngleBack; // This is a HEAVY approximation (will work fine hopefully)
    targetVelocity = (distance/Math.cos(hoodAngle))*Math.sqrt(gravity/(distance*Math.tan(hoodAngle) - targetHeight));
    // System.out.println(Math.cos(hoodAngle));
    // System.out.println(Math.tan(hoodAngle));
    // System.out.println(Math.sqrt(gravity/(distance*Math.tan(hoodAngle) - targetHeight)));

    if (lipDistance*Math.tan(hoodAngle) - (gravity*Math.pow(lipDistance, 2))/(Math.pow(targetVelocity, 2)*Math.pow(Math.cos(hoodAngle), 2)) < passableHeight) {
      System.out.println("Cannot shoot at this angle!!!");
    } else {
      leftShooter.set(-(1.54*(targetVelocity + drivetrainVelocity/Math.cos(hoodAngle)) - 4.9)/shooterMaxSpeed);
      rightShooter.set((1.54*(targetVelocity + drivetrainVelocity/Math.cos(hoodAngle)) - 4.9)/shooterMaxSpeed);
    }
    SmartDashboard.putNumber("targetVelocity", targetVelocity);
    Logger.recordOutput("targetVelocity", targetVelocity);
    SmartDashboard.putNumber("setVelocity", (1.54*(targetVelocity + drivetrainVelocity/Math.cos(hoodAngle)) - 4.9));
    Logger.recordOutput("setVelocity", (1.54*(targetVelocity + drivetrainVelocity/Math.cos(hoodAngle)) - 4.9));
    SmartDashboard.putNumber("drivetrainVelocity", drivetrainVelocity);
    SmartDashboard.putNumber("shooterAngle", hoodAngle);
    Logger.recordOutput("shooterAngle", hoodAngle);
  }

  public void setHoodAngle(double angle) {
    SmartDashboard.putNumber("targetingAngle", angle);
    Logger.recordOutput("targetingAngle", angle);
    targetPosition = (Math.sqrt(Math.pow(hoodRadius*Math.sin(angle) + shooterAxleY, 2) + Math.pow(shooterAxleX - hoodRadius*Math.cos(angle), 2)) - actuatorMinimum)/(maximumExtension);
    hoodLifter.setPosition(MathUtil.clamp(targetPosition, 0.01, 0.8));
    SmartDashboard.putNumber("targetPosition", targetPosition);
    Logger.recordOutput("targetPosition", targetPosition);
  }
}
