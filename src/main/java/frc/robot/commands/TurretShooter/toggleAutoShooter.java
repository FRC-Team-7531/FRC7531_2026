// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretShooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SS_Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class toggleAutoShooter extends Command {
  SS_Shooter shooter;
  /** Creates a new toggleAutoShooter. */
  public toggleAutoShooter(SS_Shooter ss_shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ss_shooter);
    this.shooter = ss_shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.mode == SS_Shooter.ShootMode.AUTO) {
      shooter.mode = SS_Shooter.ShootMode.MANUAL;
    } else {
      shooter.mode = SS_Shooter.ShootMode.AUTO;
    }
    System.out.println("toggling!");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
