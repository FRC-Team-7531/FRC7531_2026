// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SS_Shooter extends SubsystemBase {
  public final double shooterMaxSpeed = 17.5; //Theoretical, not gonna be exact yet

  // Constants stuff all in meters -ish
  public final double lipHeight = 1.8288;
  public final double hoopRadius = 0.5969;
  public final double gravity = 4.90335;
  public final double linearActuatorAngle = 60;
  public final double hoodArmRadius = 0.15875;
  public final double heightDifference = 0.0762;
  public final double maxExtension = 0.1397;
  public final double minimumClearance = 0.08;
  public final double nearBound = 1.50617974628;

  // Set these to change the desired trajectory
  public double targetHeight = 1.8; // h
  public double lipClearance = 0.2; // b

  // Precalculated stuff
  public final double clearanceHeight = lipHeight + lipClearance;
  public final double passableHeight = lipHeight + minimumClearance;
  public final double nearCenterDistance = hoopRadius + lipClearance;

  // Stuff to be calculated
  public double lipDistance;
  public double targetVelocity;
  public double targetAngle;
  public double targetPosition;
  public double nearDistance;

  // Hard goods
  public CANBus canivore = new CANBus("CANivore");

  public PWM leftHoodLifter = new PWM(0);
  public PWM rightHoodLifter = new PWM(1);
  public TalonFX leftShooter = new TalonFX(42, canivore);
  public TalonFX rightShooter = new TalonFX(41, canivore);

  // NetworkTable stuff
  public NetworkTableInstance inst = NetworkTableInstance.getDefault();
  public NetworkTableEntry actuatorPosition = inst.getTable("Shooter").getEntry("Actuator Position");
  public NetworkTableEntry shooterSpeed = inst.getTable("Shooter").getEntry("Shooter Speed");

  /** Creates a new SS_Shooter. */
  public SS_Shooter() {
    actuatorPosition.setDouble(0.0);
    shooterSpeed.setDouble(0.0);
    leftShooter.setNeutralMode(NeutralModeValue.Coast);
    rightShooter.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    actuatorPosition = inst.getTable("Shooter").getEntry("Actuator Position");
    shooterSpeed = inst.getTable("Shooter").getEntry("Shooter Speed");
  }

  public double calculateGoalAngle(double distance) {
    lipDistance = distance - hoopRadius;
    nearDistance = distance - hoopRadius - lipClearance;

    if (distance < nearBound) {
      targetAngle = Math.atan((lipHeight*distance)/(nearDistance*nearCenterDistance)-(targetHeight*nearDistance)/(distance*nearCenterDistance));
    } else {
      targetAngle = Math.atan((distance*clearanceHeight)/(lipDistance*hoopRadius) - (targetHeight*lipDistance)/(distance*hoopRadius));
    }

    return targetAngle;
  }

  public void setVelocity(double distance, double shooterAngle) {
    lipDistance = distance - hoopRadius;
    nearDistance = distance - hoopRadius - lipClearance;
    targetVelocity = (distance/Math.cos(shooterAngle))*Math.sqrt(gravity/(distance*Math.tan(shooterAngle) - targetHeight));

    if (lipDistance*Math.tan(shooterAngle) - (gravity*Math.pow(lipDistance, 2))/(Math.pow(targetVelocity, 2)*Math.pow(Math.cos(shooterAngle), 2)) < passableHeight) {
      System.out.println("Cannot shoot at this angle!!!");
    } else {
      leftShooter.set(targetVelocity/shooterMaxSpeed);
      rightShooter.set(targetVelocity/shooterMaxSpeed);
    }
  }

  public void setHoodAngle(double angle) {
    targetPosition = ((hoodArmRadius*Math.sin(angle) - heightDifference)/Math.sin(linearActuatorAngle))/maxExtension;
    if ((targetPosition < 0.01) || (targetPosition > 0.8)) {
      System.out.println("Clamping Hood Angle!!!");
      leftHoodLifter.setPosition(MathUtil.clamp(targetPosition, 0.01, 0.8));
      rightHoodLifter.setPosition(MathUtil.clamp(targetPosition, 0.01, 0.8));
    } else {
      leftHoodLifter.setPosition(targetPosition);
      rightHoodLifter.setPosition(targetPosition);
    }
  }
}
