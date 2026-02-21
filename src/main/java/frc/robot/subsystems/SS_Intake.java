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
  public TalonFX hotDogRoller = new TalonFX(21);
  public TalonFX roller = new TalonFX(25, "CANivore");
  public NetworkTableInstance intakeTable = NetworkTableInstance.getDefault();
  public NetworkTableEntry intakeTableEntry = intakeTable.getEntry("Encoder Position");
  public DoublePublisher doubleIntakePublisher;

  public SS_Intake() {
    setDefaultCommand(new RunCommand(() -> makeRollerStop(), this));
    pivot.setPosition(0);
    doubleIntakePublisher = intakeTable.getDoubleTopic("Encoder Position").publish();
  }

  public void makeRollerGo() {
    roller.set(0.5);
    //System.out.println("Motor In " + roller.get());
  }

  public void makeRollerStop() {
    roller.set(0.0);
    //stem.out.println("Motor Stop");
  }

  public void makeRollerSpitOut() {
    roller.set(-0.1);
    //System.out.println("Motor Out");
  }

public void intakeUnfold()  {
  double pivotAngle = pivot.getPosition().getValueAsDouble();
  //System.out.println("pivot Angle: " + pivotAngle);
  pivot.set(0.1);
}

  public void IntakePivotStop()  {
    pivot.set(0);
  }

  public void intakeFold()  {
    double pivotAngle = pivot.getPosition().getValueAsDouble();
    //System.out.println("pivot Angle: " + pivotAngle);
    pivot.set(-0.1);
  }

  public void hotDogRollersOn() {
    hotDogRoller.set(-0.5);
  }
  
  public void hotDogRollersOff() {
    hotDogRoller.set(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    doubleIntakePublisher.set(pivot.getPosition().getValueAsDouble());
  }
}
