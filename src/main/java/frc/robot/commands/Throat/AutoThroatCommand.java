// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Throat;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SS_Hopper;
import frc.robot.subsystems.SS_Intake;
import frc.robot.subsystems.SS_Throat;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoThroatCommand extends Command {
  public SS_Throat throat;
  public SS_Hopper hopper;
  public SS_Intake intake;
  public Timer timer2 = new Timer();
  public double timerSeconds = 10;
  public int counter = 0;

  /** Creates a new startThroat. */
  public AutoThroatCommand(SS_Throat ss_throat, SS_Hopper hopper, SS_Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ss_throat);
    this.throat = ss_throat;

    addRequirements(hopper);
    this.hopper = hopper;

    addRequirements(intake);
    this.intake = intake;
    
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer2.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    throat.throatMotor.set(0.7);
    hopper.hotDogRollersOn();
    hopper.carWashOn();
    //intake.intakeRollersOn(1);
    // System.out.println("Start Throat");
    // System.out.println(throat.throatMotor.get());
    if(timer2.hasElapsed(1))
    {
      hopper.carWashBack();
      hopper.hotDogRollersBack();
    }
    if (timer2.hasElapsed(1.5)) {
      hopper.carWashOn();
      hopper.hotDogRollersOn();
    }
    if(timer2.hasElapsed(5) && counter < 10)
    {
      if (intake.pivot.getPosition().getValueAsDouble() > 13) {
        intake.intakeFold();
        counter++;
      }
      if (intake.pivot.getPosition().getValueAsDouble() < 7) {
        intake.intakeUnfold();
        counter++;
      }
      
    }
    else
    {
      intake.PivotStop();
    }
    
    System.out.println(counter);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //throat.timer.stop();
    //throat.timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return true;
    if (timer2.hasElapsed(timerSeconds)) {
      timer2.reset();
      timer2.stop();
      hopper.hotDogRollersOff(); 
      throat.throatMotor.set(0);
      hopper.carWashOff();
      return true;
    }
    else { 
      return false;
    }
  }
}



