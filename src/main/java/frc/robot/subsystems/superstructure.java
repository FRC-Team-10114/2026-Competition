package frc.robot.subsystems;

import java.util.List;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain.AutoAlign;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper.HopperSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;

import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.util.FIeldHelper.AllianceFlipUtil;
import frc.robot.util.FIeldHelper.FieldTagMap;

public class superstructure extends SubsystemBase {

    private final CommandSwerveDrivetrain drive;

    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private final HopperSubsystem hopper;
    private final AutoAlign autoAlign;


    public superstructure(
            CommandSwerveDrivetrain drive,
            ShooterSubsystem shooter,
            IntakeSubsystem intake, 
            HopperSubsystem hopper,
            AutoAlign autoAlign
    ) {
        this.drive = drive;
        this.shooter = shooter;
        this.intake = intake;
        this.hopper = hopper;
        this.autoAlign = autoAlign;
    }

    // Intake Methods

    public Command intakeCommand() {
        return Commands.startEnd(
            intake::intake, 
            intake::back);
    }

    // Hopper Methods

    public Command warmUpCommand() {
        return Commands.runOnce(hopper::warmUp);
    }

    public Command stopWashCommand() {
        return Commands.run(hopper::stopSpin);
    }

    public Command loadCommand() {
        return Commands.startEnd(
            hopper::load, 
            hopper::stopTrigger);
    }
    public Command DriveToTrench(){
       return this.autoAlign.DriveToTrench();
    }
    @Override
    public void periodic() {
    }
}
