// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.*;
import java.io.*;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
//Idk what needs to be imported so I imported a bunch of stuff I found

public class SS_Hanger extends SubsystemBase {
  // height is measured from ground to top of bar + 2.108

  // intitialize motors
  // public TalonFX motor = new TalonFX(0);

  public final double tier1height = 29.108;
  public final double tier2height = 47.108;
  public final double tier3height = 66.108;
  // should note that the hook is about 0.21 inches, so possibly add that but this
  // measurements may not be used at all

  public TalonFX m_hangLeft = new TalonFX(999, "CANivore");
  public TalonFX hangRight = new TalonFX(999, "CANivore");
  public NetworkTableInstance hangerTable = NetworkTableInstance.getDefault();
  public NetworkTableEntry hangerTableEntry = hangerTable.getEntry("Encoder Position");
  public DoublePublisher hangerPublisher;

  /** Creates a new SS_Hanger. */
  public SS_Hanger() {
    m_hangLeft.setPosition(0);
    hangRight.setPosition(0);
    hangerPublisher = hangerTable.getDoubleTopic("Encoder Position").publish();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    hangerPublisher.set(m_hangLeft.getPosition().getValueAsDouble());
    hangerPublisher.set(hangRight.getPosition().getValueAsDouble());
  }

  public void HangerStop() {
    m_hangLeft.set(0);
    hangRight.set(0);
  }

  public void HangLeft() {
    double leftPosition = m_hangLeft.getPosition().getValueAsDouble();

    m_hangLeft.set(0.1);
  }

  public void HangRight() {
    double rightPosition = m_hangLeft.getPosition().getValueAsDouble();

    hangRight.set(0.1);
  }

  public void HangReturn() {
    double leftPosition = m_hangLeft.getPosition().getValueAsDouble();
    double rightPosition = hangRight.getPosition().getValueAsDouble();
    m_hangLeft.set(-0.1);
    hangRight.set(-0.1);
  }

}
