// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Auto;
import frc.robot.subsystems.RobotStatus;
import frc.robot.subsystems.superstructure;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drivetrain.SwerveDrivetrainTest;
import frc.robot.subsystems.Drivetrain.TunerConstants;
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
import frc.robot.subsystems.Shooter.Hood.HoodIONEO;
import frc.robot.subsystems.Shooter.Turret.TurretHardware;
import frc.robot.subsystems.Shooter.Turret.TurretIO;
import frc.robot.subsystems.Vision.Limelight;
import frc.robot.subsystems.RobotStatus;

public class RobotContainer {

    private double MaxSpeed = (4 / 5.47) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts
                                                                                               // desired top speed
    private double MaxTeleOpSpeed = MaxSpeed;
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxTeleOpSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Limelight limelight = new Limelight(drivetrain, "limelight-left");

    private final Field2d field;

    private final SwerveDrivetrainTest[] tests = new SwerveDrivetrainTest[4];

    private final Auto auto = new Auto(drivetrain);

    private final TurretIO turret = new TurretHardware();

    private final FlywheelIO flywheel = new FlywheelHardware();

    private final HoodIO hood = new HoodIONEO();

    private final ArmIO arm = new ArmIOTalon();

    private final RollerIO roller = new RollerIOTalon();

    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(arm, roller);
    public final RobotStatus robotStatus = new RobotStatus(drivetrain);

    private final ShooterCalculator shooterCalculator = new ShooterCalculator(drivetrain, robotStatus);

    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(hood, flywheel, turret, shooterCalculator,
            drivetrain, robotStatus);

    private final superstructure superstructure = new superstructure(drivetrain, shooterSubsystem, intakeSubsystem,
            robotStatus);

    public RobotContainer() {

        // Swerve Drivetrain Current & Voltage Test
        for (int i = 0; i < 4; i++)
            this.tests[i] = new SwerveDrivetrainTest(drivetrain, i);

        field = new Field2d();
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

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        // FollowPathCommand.warmupCommand().schedule(); (Deprecated)
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(

                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(
        // () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
        // -joystick.getLeftX()))));

        // joystick.povUp().whileTrue(drivetrain.applyRequest(() ->
        // forwardStraight.withVelocityX(0.5).withVelocityY(0)));
        // joystick.povDown()
        // .whileTrue(drivetrain.applyRequest(() ->
        // forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        // joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // drivetrain.registerTelemetry(logger::telemeterize);
        // joystick.rightBumper().onTrue(drivetrain.runOnce(drivetrain::resetPosetotest));
        // joystick.a().whileTrue(this.superstructure.DriveToTrench());

        joystick.povUp().whileTrue(new InstantCommand(() -> shooterSubsystem.hoodUp()));
        joystick.povDown().whileTrue(new InstantCommand(() -> shooterSubsystem.hoodDown()));

        joystick.y().onTrue(new InstantCommand(() -> shooterSubsystem.flywheelup()));
        joystick.a().onTrue(new InstantCommand(() -> shooterSubsystem.flywheeldown()));

    }

    public Command getAutonomousCommand() {
        return this.auto.auto();

    }
}