// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Hang.alignTower;
import frc.robot.commands.Intake.foldIntake_cmd;
import frc.robot.commands.Intake.intake_cmd;
import frc.robot.commands.Intake.outtake_cmd;
import frc.robot.commands.Intake.rollersOff_cmd;
import frc.robot.commands.Intake.rollersOn_cmd;
import frc.robot.commands.Intake.unfoldIntake_cmd;
import frc.robot.commands.Throat.startThroat;
import frc.robot.commands.Throat.stopThroat;
import frc.robot.commands.TurretShooter.aimTurretToTarget;
import frc.robot.commands.TurretShooter.manualShooter;
import frc.robot.commands.TurretShooter.manualTurret;
import frc.robot.commands.TurretShooter.stopTurret;
import frc.robot.commands.TurretShooter.moveActuator;
import frc.robot.commands.TurretShooter.stopShooter;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SS_Drivetrain;
import frc.robot.subsystems.SS_Hopper;
import frc.robot.subsystems.SS_Intake;
import frc.robot.subsystems.SS_Shooter;
import frc.robot.subsystems.SS_Throat;
import frc.robot.subsystems.SS_Turret;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);

    public final SS_Drivetrain drivetrain = TunerConstants.createDrivetrain();

    public final SS_Shooter shooter = new SS_Shooter();
    public final SS_Turret turret = new SS_Turret();
    public final SS_Throat throat = new SS_Throat();
    public aimTurretToTarget aimCommand = new aimTurretToTarget(drivetrain, turret);
    public Command drivetrainDefault = drivetrain.applyRequest(() ->
        drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
             .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
             .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    );
    public alignTower alignTowerCommand = new alignTower(drivetrain);
    public manualTurret turretForward = new manualTurret(turret).withSpeed(0.1);
    public manualTurret turretReverse = new manualTurret(turret).withSpeed(-0.1);
    public stopTurret stopCommand = new stopTurret(turret);
    public manualShooter shootCommand = new manualShooter(shooter);
    public moveActuator moveActuatorCommand = new moveActuator(shooter);
    public stopShooter stopShooterCommand = new stopShooter(shooter);
    public stopThroat stopThroatCommand = new stopThroat(throat);
    public startThroat startThroatCommand = new startThroat(throat);


    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    private final SS_Intake intake = new SS_Intake();
    private final SS_Hopper hopper = new SS_Hopper();

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }



    
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        turret.setDefaultCommand(stopCommand);
        shooter.setDefaultCommand(stopShooterCommand);
        throat.setDefaultCommand(stopThroatCommand);

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        joystick2.a().onTrue(new intake_cmd(intake));
        joystick2.b().onTrue(new outtake_cmd(intake));
        joystick2.rightBumper().onTrue(new unfoldIntake_cmd(intake));
        joystick2.leftBumper().onTrue(new foldIntake_cmd(intake));
        joystick2.x().onTrue(new rollersOn_cmd(intake));
        joystick2.y().onTrue(new rollersOff_cmd(intake));
        
        joystick2.povLeft().whileTrue(turretForward);
        joystick2.povRight().whileTrue(turretReverse);
        joystick2.leftTrigger().whileTrue(shootCommand);
        joystick2.rightTrigger().whileTrue(startThroatCommand);

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from ;;t;he auto chooser */
        return autoChooser.getSelected();
    }
}
;