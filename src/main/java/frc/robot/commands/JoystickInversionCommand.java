// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SS_Vision;

/** A value of 1 that every time the command is called changes polarity. */
public class JoystickInversionCommand extends Command {
  /** A value of 1 that every time the command is called changes polarity. */
  public JoystickInversionCommand(SS_Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision);
  }

  public double joystickInvert = 1;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    joystickInvert = -joystickInvert;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}



