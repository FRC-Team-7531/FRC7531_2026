// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SS_Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class outtake_cmd extends Command {
  /** Creates a new foldIntake. */
  public Timer timerIntake = new Timer();
  public SS_Intake intake;

  public outtake_cmd(SS_Intake ss_intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ss_intake);
    this.intake = ss_intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timerIntake.start();
    System.out.println("Out");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.makeRollerSpitOut();
    System.out.println("Out Running " + timerIntake.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timerIntake.hasElapsed(5)) {
      timerIntake.restart();
      timerIntake.stop();
      intake.makeRollerStop();
      return true;
    }
    else {
      return false;
    }
  }
}
