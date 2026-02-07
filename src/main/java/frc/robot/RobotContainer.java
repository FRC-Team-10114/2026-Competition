// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Auto;
import frc.robot.subsystems.superstructure;
import frc.robot.subsystems.Drivetrain.AutoAlign;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drivetrain.SwerveDrivetrainTest;
import frc.robot.subsystems.Drivetrain.TunerConstants;
import frc.robot.subsystems.Hopper.HopperSubsystem;
import frc.robot.subsystems.Hopper.Spindexer.SpindexerIO;
import frc.robot.subsystems.Hopper.Spindexer.SpindexerIOHardware;
import frc.robot.subsystems.Hopper.Trigger.TriggerIO;
import frc.robot.subsystems.Hopper.Trigger.TriggerIOHardware;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Intake.Arm.ArmIO;
import frc.robot.subsystems.Intake.Arm.ArmIOTalon;
import frc.robot.subsystems.Intake.Roller.RollerIO;
import frc.robot.subsystems.Intake.Roller.RollerIOTalon;
import frc.robot.subsystems.Shooter.ShooterCalculator;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelHardware;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIO;
import frc.robot.subsystems.Shooter.Hood.HoodIO;
import frc.robot.subsystems.Shooter.Hood.HoodTalon;
import frc.robot.subsystems.Shooter.Turret.TurretIOTalon;
import frc.robot.subsystems.Shooter.Turret.TurretIO;
import frc.robot.subsystems.Vision.Limelight;
import frc.robot.subsystems.Vision.PhotonVision;
import frc.robot.util.FMS.Signal;
import frc.robot.util.RobotStatus.RobotStatus;

public class RobotContainer {

    private double MaxSpeed = (4 / 5.47) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxTeleOpSpeed = MaxSpeed;
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxTeleOpSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final SwerveDrivetrainTest[] tests = new SwerveDrivetrainTest[4];

    public final RobotStatus robotStatus = new RobotStatus(drivetrain);

    public final Limelight limelight = new Limelight(drivetrain, "limelight-left");
    public final PhotonVision photonVision = new PhotonVision(drivetrain, Constants.PhotonVisionConstants.newCam);

    private final Field2d field = new Field2d();

    private final Auto auto = new Auto(drivetrain);

    private final TurretIO turret = new TurretIOTalon();
    private final FlywheelIO flywheel = new FlywheelHardware();
    private final HoodIO hood = new HoodTalon();
    private final ShooterCalculator shooterCalculator = new ShooterCalculator(drivetrain, robotStatus);
    private final ShooterSubsystem shooter = new ShooterSubsystem(hood, flywheel, turret, shooterCalculator, drivetrain,
            robotStatus);

    private final ArmIO arm = new ArmIOTalon();
    private final RollerIO roller = new RollerIOTalon();
    private final IntakeSubsystem intake = new IntakeSubsystem(arm, roller);

    private final SpindexerIO washingMechine = new SpindexerIOHardware();
    private final TriggerIO trigger = new TriggerIOHardware();
    private final HopperSubsystem hopper = new HopperSubsystem(trigger, washingMechine);

    private final AutoAlign autoAlign = new AutoAlign(drivetrain, robotStatus);

    private final superstructure superstructure = new superstructure(drivetrain, shooter, intake,
            hopper, autoAlign);
    public final Signal signal = new Signal();

    public RobotContainer() {

        // Swerve Drivetrain Current & Voltage Test
        for (int i = 0; i < 4; i++)
            this.tests[i] = new SwerveDrivetrainTest(drivetrain, i);

        configureBindings();

        configureEvents();

        log();

        // Warmup PathPlanner to avoid Java pauses
        // FollowPathCommand.warmupCommand().schedule(); (Deprecated)
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    public PhotonVision getphotonVision() {
        return this.photonVision;
    }

    public Auto getauto() {
        return this.auto;
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));
        drivetrain.registerTelemetry(logger::telemeterize);

        // joystick.start().onTrue(drivetrain.runOnce(drivetrain::resetPosetotest));

        // joystick.rightBumper().whileTrue(this.superstructure.DriveToTrench());

        // joystick.leftBumper().whileTrue(this.superstructure.shoot());

        joystick.a().onTrue(
                Commands.runOnce(() -> this.shooter.hoodUp(), this.shooter));

        joystick.b().onTrue(
                Commands.runOnce(() -> this.shooter.hoodDown(), this.shooter));

        joystick.leftBumper().onTrue(
                Commands.runOnce(() -> this.shooter.flywheelup(), this.shooter));

        joystick.rightBumper().onTrue(
                Commands.runOnce(() -> this.shooter.flywheeldown(), this.shooter));
        // // sysidTest();
    }

    public Command getAutonomousCommand() {
        return this.auto.auto();

    }

    public void sysidTest() {
        joystick.povUp().whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        joystick.povDown()
                .whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

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

    private void configureEvents() {
        robotStatus.TriggerNeedResetPoseEvent(photonVision::NeedResetPoseEvent);
        signal.TargetInactive(shooter::FalseTargetactive);
        signal.Targetactive(shooter::TrueTargetactive);
        superstructure.TriggerShootingStateTrue(shooter::TrueIsshooting);
        superstructure.TriggerShootingStateFalse(shooter::FalseIsshooting);
    }
}