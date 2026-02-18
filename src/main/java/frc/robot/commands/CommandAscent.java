// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//this is for hanging
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CommandAscent extends Command {

  private int ascentLevel = 0;
  //if any errors happen, it will just default to not hanging at all, because I feel that prevents issues 

  /** Creates a new CommandAscent. */
  public CommandAscent(int ascentLevelPick) {
  //int determines what level is wanted for hanging
    if ((ascentLevelPick != 1) || (ascentLevelPick != 2) ||(ascentLevelPick != 3)) {
      ascentLevel = 0;
    } else {
    ascentLevel = ascentLevelPick;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize () {
      //error handling determiend in command creation

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      //do stuff with the motors i guess here
      if (ascentLevel > 0) {
      //do tier 1
      

      if (ascentLevel > 1) {
        //do tier 2
          
        if (ascentLevel > 2) {
          //do tier 3
        }
      }
    }
      //this.isFinished();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      //failsafe of some sort, such as let robot down gently, lock in current place to hang, or something
      //climbingMotor1.position = currentPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
