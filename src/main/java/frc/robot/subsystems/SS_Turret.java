// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SS_Turret extends SubsystemBase {
  public CANBus canivore = new CANBus("CANivore");
  public CANBus rio = new CANBus("rio");
  public CANcoder encoder = new CANcoder(44, rio);
  public TalonFX turretMotor = new TalonFX(40, rio);
  public Pigeon2 pidgey = new Pigeon2(0, canivore);
  public final double leftMaximum = 0.5;
  public final double rightMaximum = -0.25;
  Translation2d position;

  /** Creates a new SS_Turret. */
  public SS_Turret() {
    encoder.setPosition(0);
    turretMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("TurretRotation", -36*encoder.getPosition().getValueAsDouble());
    getTurretRotation();
  }

  // All of these inputs should be relative to the turret, not the motor that drives it

  public double getTurretRotation() {
    return -0.1*encoder.getPosition().getValueAsDouble();
  }

  public Translation2d getTurretPosition(double yaw) {
    position = new Translation2d(0.2032*Math.cos(yaw), 0.2032*Math.sin(yaw));
    return position;
  }

  public void setRawSpeed(double speed) {
    turretMotor.set(speed);
  }
}
