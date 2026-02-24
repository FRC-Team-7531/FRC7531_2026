// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Throat;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SS_Hopper;
import frc.robot.subsystems.SS_Throat;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoThroat_cmd extends Command {
  public SS_Throat throat;
  public SS_Hopper hopper;
  public Timer timer2 = new Timer();
  public double timerSeconds = 5;

  /** Creates a new startThroat. */
  public AutoThroat_cmd(SS_Throat ss_throat, SS_Hopper hopper) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ss_throat);
    this.throat = ss_throat;

    addRequirements(hopper);
    this.hopper = hopper;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer2.start();
    throat.throatMotor.set(0.7);
    hopper.hotDogRollersOn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    throat.timer.stop();
    throat.timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer2.hasElapsed(timerSeconds)) {
      timer2.reset();
      timer2.stop();
      hopper.hotDogRollersOff();
      throat.throatMotor.set(0);
      return true;
    }
    else {
      return false;
    }
  }
}
