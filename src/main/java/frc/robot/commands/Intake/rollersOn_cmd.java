// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SS_Hopper;
import frc.robot.subsystems.SS_Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class rollersOn_cmd extends Command {
  /** Creates a new rollersOn_cmd. */
  public Timer timer = new Timer();
  public SS_Hopper hopper;
  public NetworkTableInstance inst = NetworkTableInstance.getDefault();
  public NetworkTableEntry timerEntry = inst.getTable("HotDog").getEntry("HotDogTimer");

  public rollersOn_cmd(SS_Hopper hopper) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hopper);
    this.hopper = hopper;
    timerEntry.setDouble(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hopper.hotDogRollersOn();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    timerEntry.setDouble(timer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.hasElapsed(5)) {
      timer.reset();
      timer.stop();
      hopper.hotDogRollersOff();
      return true;
    }
    else {
    return false;
    }
  }
}
