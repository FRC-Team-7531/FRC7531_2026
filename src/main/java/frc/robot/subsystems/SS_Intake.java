// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class SS_Intake extends SubsystemBase {
  /** Creates a new SS_Intake. */
  public TalonFX pivot = new TalonFX(20);
  public TalonFX hotDogRoller = new TalonFX(21);
  public TalonFX roller = new TalonFX(25);
  public SS_Intake() {
    setDefaultCommand(new RunCommand(() -> makeRollerStop(), this));
  }

  public void makeRollerGo() {
    roller.set(1);
    System.out.println("Motor In " + roller.get());
  }

  public void makeRollerStop() {
    roller.set(0.0);
    System.out.println("Motor Stop");
  }

  public void makeRollerSpitOut() {
    roller.set(-1);
    System.out.println("Motor Out");
  }

public void intakeUnfold()  {
   pivot.set(0.25);
  }

  public void IntakePivotStop()  {
    pivot.set(0);
  }

  public void intakeReverse()  {
    pivot.set(-0.25);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
