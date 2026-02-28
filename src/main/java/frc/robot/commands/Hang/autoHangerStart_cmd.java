// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hang;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SS_Hanger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class autoHangerStart_cmd extends Command {
  /** Creates a new autoHangerStart_cmd. */
  public SS_Hanger hanger;
  public double hanger_rotation = 10;

  public autoHangerStart_cmd(SS_Hanger ss_hanger) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ss_hanger);
    this.hanger = ss_hanger;
  }

  public autoHangerStart_cmd() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hanger.HangLeft();
    hanger.HangRight();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (hanger.m_hangLeft.getPosition().getValueAsDouble() >= hanger_rotation) {
      hanger.HangerStop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
