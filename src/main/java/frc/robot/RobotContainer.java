// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.superstructure;
import frc.robot.subsystems.Dashboard.Dashboard;
import frc.robot.subsystems.Drivetrain.AutoAlign;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drivetrain.SwerveDrivetrainTest;
import frc.robot.subsystems.Drivetrain.TunerConstants;
import frc.robot.subsystems.Hopper.HopperSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Vision.PhotonVision;
import frc.robot.util.FMS.Signal;
import frc.robot.util.RobotStatus.RobotStatus;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {

    // Constants for tuning
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxTeleOpSpeed = MaxSpeed;
    private final double MaxAngularRate = RotationsPerSecond.of(1.25).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxTeleOpSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandXboxController joystick = new CommandXboxController(0);

    final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final SwerveDrivetrainTest[] tests = new SwerveDrivetrainTest[4];
    private final Telemetry logger = new Telemetry(this.drivetrain.getState());
    private final RobotStatus robotStatus = new RobotStatus(drivetrain);

    private final AutoAlign autoAlign = new AutoAlign(drivetrain, robotStatus);

    public final PhotonVision photonVision = new PhotonVision(drivetrain,
            Constants.PhotonVisionConstants.cameraTransforms);

    private final Field2d field = new Field2d();

    private final ShooterSubsystem shooter = ShooterSubsystem.create(drivetrain, robotStatus);
    private final IntakeSubsystem intake = IntakeSubsystem.create();
    private final HopperSubsystem hopper = HopperSubsystem.create();

    private final LED led = new LED();

    private final superstructure superstructure = new superstructure(drivetrain, shooter, intake, hopper, led,
            autoAlign);

    public final Signal signal = new Signal();

    private final Dashboard dashboard = new Dashboard(signal);

    // public final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        // Swerve Drivetrain Current Test
        for (int i = 0; i < 4; i++)
            this.tests[i] = new SwerveDrivetrainTest(drivetrain, i);

        configureBindings();

        configureEvents();

        log();

        NamedCommands.registerCommand("intakeDown", superstructure.intake());
        NamedCommands.registerCommand("intakestop", superstructure.stopintake());
        NamedCommands.registerCommand("HoodDown", Commands.runOnce(() -> robotStatus.SetSafeHood(true), robotStatus));
        NamedCommands.registerCommand("StopHoodDown",
                Commands.runOnce(() -> robotStatus.SetSafeHood(false), robotStatus));
        NamedCommands.registerCommand("shoot", superstructure.autoshooter().withTimeout(2.0));
        NamedCommands.registerCommand("stopshoot", superstructure.stopShoot());
        NamedCommands.registerCommand("isIn",
                Commands.run(() -> System.out.println("Stop!!!!!!!")).until(robotStatus::isInTrench));

        boolean isCompetition = true;

        // Build an auto chooser. This will use Commands.none() as the default option.
        // As an example, this will only show autos that start with "comp" while at
        // // competition as defined by the programmer
        // autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        //         (stream) -> isCompetition
        //                 ? stream.filter(auto -> auto.getName().startsWith("comp"))
        //                 : stream);

        // SmartDashboard.putData("Auto Chooser", autoChooser);

        // Warmup PathPlanner to avoid Java pauses
        // FollowPathCommand.warmupCommand().schedule(); (Deprecated)
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

        // drivetrain.runOnce(drivetrain::resetPosetotest);
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-joystick.getLeftY() * MaxTeleOpSpeed)
                        .withVelocityY(-joystick.getLeftX() * MaxTeleOpSpeed)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));
        drivetrain.registerTelemetry(logger::telemeterize);

        // joystick.y().onTrue(drivetrain.runOnce(drivetrain::resetPosetotest));

        // joystick.leftBumper().whileTrue(this.superstructure.DriveToTrench());

        joystick.leftBumper().whileTrue(Commands.runOnce(() -> robotStatus.SetSafeHood(true), robotStatus))
                .onFalse(Commands.runOnce(() -> robotStatus.SetSafeHood(false), robotStatus));

        joystick.leftTrigger().whileTrue((superstructure.intake()))
                .onFalse(superstructure.stopintake());
        joystick.rightTrigger().whileTrue(this.superstructure.shootCommand())
                .onFalse(this.superstructure.stopShoot());

        // ---------------------------------------test
        // Method-------------------------------------------------

        // joystick.a().onTrue(this.shooter.sysid());

        // joystick.leftBumper().whileTrue(this.superstructure.shoot());

        // joystick.x().whileTrue(
        // Commands.runOnce(() -> this.shooter.hoodUp(), this.shooter));

        // joystick.b().whileTrue(
        // Commands.runOnce(() -> this.shooter.hoodDown(), this.shooter));

        // joystick.leftBumper().whileTrue(Commands.runOnce(() ->
        // this.shooter.flywheelup(), this.shooter));

        // joystick.rightBumper().whileTrue(
        // Commands.runOnce(() -> this.shooter.flywheeldown(), this.shooter));

        // joystick.leftBumper().onTrue(
        // Commands.run(() -> robotStatus.xtrue()))
        // .onFalse(Commands.run(robotStatus::xfalse));
        // // sysidTest();
        // joystick.leftBumper().whileTrue( Commands.run(() -> this.shooter.turretup(),
        // this.shooter));
        // joystick.rightBumper().whileTrue( Commands.run(() ->
        // this.shooter.turretdown(), this.shooter));

        // joystick.a().onTrue(superstructure.intake())
        // .onFalse(superstructure.stopintake());

        // -----------------------sysid Method-------------------------------
        // joystick.x().onTrue(this.shooter.startCommand());
        // joystick.a().onTrue(this.shooter.stopCommand());
        // joystick.rightBumper().onTrue(this.shooter.sysIdTest());
    }

    private void configureEvents() {
        robotStatus.TriggerNeedResetPoseEvent(photonVision::NeedResetPoseEvent);
        robotStatus.TriggerInTrench(shooter::TrueInTrench);
        robotStatus.TriggerNotInTrench(shooter::FalsInTrench);
        signal.TargetInactive(shooter::FalseTargetactive);
        signal.Targetactive(shooter::TrueTargetactive);
        superstructure.TriggerShootingStateTrue(shooter::TrueIsshooting);
        superstructure.TriggerShootingStateFalse(shooter::FalseIsshooting);
    }

    public PhotonVision getPhotonVisionInstance() {
        return this.photonVision;
    }


    public CommandSwerveDrivetrain getDrivetrainInstance() {
        return this.drivetrain;
    }

    public Command getAutonomousCommand() {
        return null;

    }

    public void sysidTest() {
        joystick.povUp().whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        joystick.povDown()
                .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    }

    public void log() {
        SmartDashboard.putData("Field", field);

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });

    }
}