// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SS_Hopper extends SubsystemBase {
  /** Creates a new SS_Hopper. */
  private TalonFX m_rollers = new TalonFX(21);
  public SS_Hopper() {}

  public void RollersOn(double motorSpeed)
  {
    m_rollers.set(motorSpeed);
  }

  public void StopRollers()
  {
    m_rollers.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
