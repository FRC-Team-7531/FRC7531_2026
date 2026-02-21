// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class SS_Intake extends SubsystemBase {
  /** Creates a new SS_Intake. */
  public TalonFX pivot = new TalonFX(20, "CANivore");
  public TalonFX roller = new TalonFX(25, "CANivore");
  public NetworkTableInstance intakeTable = NetworkTableInstance.getDefault();
  public NetworkTableEntry intakeTableEntry = intakeTable.getEntry("Encoder Position");
  public DoublePublisher doubleIntakePublisher;

  public SS_Intake() {
    setDefaultCommand(new RunCommand(() -> stopIntakeRollers(), this));
    pivot.setPosition(0);
    doubleIntakePublisher = intakeTable.getDoubleTopic("Encoder Position").publish();
  }

  public void intakeRollersOn(double speed) {
    roller.set(speed);
  }

  public void stopIntakeRollers() {
    roller.set(0.0);
  }


public void intakeUnfold()  {
  double pivotAngle = pivot.getPosition().getValueAsDouble();
  pivot.set(0.1);
}

  public void PivotStop()  {
    pivot.set(0);
  }

  public void intakeFold()  {
    double pivotAngle = pivot.getPosition().getValueAsDouble();
    pivot.set(-0.1);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    doubleIntakePublisher.set(pivot.getPosition().getValueAsDouble());
  }
}
