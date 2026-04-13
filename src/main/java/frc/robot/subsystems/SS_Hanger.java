// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SS_Hanger extends SubsystemBase {
  // height is measured from ground to top of bar + 2.108

  // intitialize motors
  // public TalonFX motor = new TalonFX(0);

  public final double tier1height = 29.108;
  public final double tier2height = 47.108;
  public final double tier3height = 66.108;
  // should note that the hook is about 0.21 inches, so possibly add that but this
  // measurements may not be used at all


  public TalonFX m_hangLeft = new TalonFX(51, "CANivore"); // long
  public TalonFX m_hangRight = new TalonFX(50, "CANivore"); // short
  public NetworkTableInstance hangerTable = NetworkTableInstance.getDefault();
  public NetworkTableEntry hangerTableEntry = hangerTable.getEntry("Encoder Position");
  public DoublePublisher hangerPublisher;
  public NetworkTableInstance inst= NetworkTableInstance.getDefault();
  public NetworkTableEntry rightPositionEntry = inst.getTable("Hanger").getEntry("Right Position");
  public NetworkTableEntry leftPositionEntry = inst.getTable("Hanger").getEntry("Left Position");

  /** Creates a new SS_Hanger. */
  public SS_Hanger() {
    m_hangLeft.setPosition(0);
    m_hangRight.setPosition(0);
    hangerPublisher = hangerTable.getDoubleTopic("Encoder Position").publish();
    rightPositionEntry.setDouble(0);
    leftPositionEntry.setDouble(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    hangerPublisher.set(m_hangLeft.getPosition().getValueAsDouble());
    hangerPublisher.set(m_hangRight.getPosition().getValueAsDouble());
  }

  public void HangerStop() {
    m_hangLeft.set(0);
    m_hangRight.set(0);
  }

  public void HangLeft() {
    double leftPosition = m_hangLeft.getPosition().getValueAsDouble();
    //SmartDashboard.getPosition(leftPosition);
    m_hangLeft.set(0.4);
    rightPositionEntry.setDouble(leftPosition);
  }

  public void HangRight() {
    double rightPosition = m_hangLeft.getPosition().getValueAsDouble();
  
    m_hangRight.set(0.4);
    leftPositionEntry.setDouble(rightPosition);
    
  }

  public void HangReturn() {
    m_hangLeft.set(-0.6);
    m_hangRight.set(-0.6);
  }

}
