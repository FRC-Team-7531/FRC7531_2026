// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

public class SS_Intake extends SubsystemBase {
  /** Creates a new SS_Intake. */
  public TalonFX m_pivot = new TalonFX(20);
  public TalonFX m_intake = new TalonFX(22);

   public void IntakeOn(double motorSpeed)
  {
    m_intake.set(motorSpeed);
  }

  public void StopIntake()
  {
    m_intake.set(0);
  }

  public void PivotOn(double motorSpeed)
  {
    m_pivot.set(motorSpeed);
  }

  public void StopPivot()
  {
    m_pivot.set(0);
  }

  public SS_Intake() {
    setDefaultCommand(new RunCommand(() -> StopIntake(), this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
