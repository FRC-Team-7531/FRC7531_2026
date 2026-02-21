// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hang;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SS_Hanger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HangLevel2_cmd extends Command {
  /** Creates a new HangLevel1_cmd. */
  public SS_Hanger hanger;

  public HangLevel2_cmd(SS_Hanger ss_hanger) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ss_hanger);
    this.hanger = ss_hanger;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hanger.HangRight();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hanger.HangRight();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // idk 999 is my placeholder for all of this
    if ((hanger.hangRight.getPosition().getValueAsDouble() < 999)) {
      hanger.HangerStop();
      return true;
    } else {
      return false;
    }
  }
}
