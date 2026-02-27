// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SS_Hopper;
import frc.robot.subsystems.SS_Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class intakeToggle_cmd extends Command {
  /** Creates a new foldIntake. */
  public SS_Intake intake;
  public SS_Hopper hopper;
  public boolean toggle = false;
  public XboxController controller = new XboxController(0);
  public boolean pressed = false;
  public Timer timer;

  public intakeToggle_cmd(SS_Intake ss_intake, SS_Hopper ss_hopper) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ss_intake);
    this.intake = ss_intake;
    addRequirements(ss_hopper);
    this.hopper = ss_hopper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (toggle) {
      intake.intakeRollersOn(0);
      toggle = false;
    }
    else {
      intake.intakeRollersOn(1);
      toggle = true;
    }
    pressed = true;
    timer.reset();
    timer.start();
    hopper.hotDogRollersOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controller.rightTrigger(1, null).getAsBoolean() && pressed == false && toggle == true) {
      intake.intakeRollersOn(0);
      toggle = false;
      pressed = true;
      timer.reset();
      timer.start();
    }
    if (controller.rightTrigger(1, null).getAsBoolean() && pressed == false && toggle == false){
      intake.intakeRollersOn(1);
      toggle = true;
      pressed = true;
      timer.reset();
      timer.start();
    }
    if (controller.rightTrigger(0, null).getAsBoolean()) {
      pressed = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.hasElapsed(5)) {
      hopper.hotDogRollersOff();
      timer.reset();
      return true;
    }
    return false;
  }
}
