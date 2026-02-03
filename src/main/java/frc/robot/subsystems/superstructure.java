package frc.robot.subsystems;

import java.util.List;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    public final RobotStatus robotStatus;

    public superstructure(
            CommandSwerveDrivetrain drive,
            ShooterSubsystem shooter,
            IntakeSubsystem intake, 
            HopperSubsystem hopper,
            RobotStatus robotStatus
    ) {
        this.drive = drive;
        this.shooter = shooter;
        this.intake = intake;
        this.hopper = hopper;
        this.robotStatus = robotStatus;
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


    // Pose Alignments Methods

    public Pose2d ToTrenchPose() {
        Pose2d[] selectedTrench;
        if (robotStatus.getVerticalSide() == RobotStatus.VerticalSide.TOP) {
            selectedTrench = FieldTagMap.getLeftTrenchPoses();
        } else {
            selectedTrench = FieldTagMap.getRightTrenchPoses();
        }
        Pose2d finalTarget;
        if (robotStatus.getArea() == RobotStatus.Area.CENTER) {
            finalTarget = selectedTrench[0];
        } else {
            finalTarget = selectedTrench[1];
        }

        return AllianceFlipUtil.apply(finalTarget);

    }

    public Command DriveToTrench() {
        List<Rotation2d> snapAngles = List.of(
                Rotation2d.fromDegrees(0),
                Rotation2d.fromDegrees(90),
                Rotation2d.fromDegrees(180),
                Rotation2d.fromDegrees(-90));

        // 使用 defer: 確保「按下按鈕的那一瞬間」才計算機器人要去哪
        return Commands.defer(() -> {

            Pose2d rawTargetPose = ToTrenchPose();

            Rotation2d currentRotation = drive.getRotation();
            Rotation2d bestAngle = snapAngles.get(0);
            double minError = Double.MAX_VALUE;

            for (Rotation2d target : snapAngles) {
                double error = Math.abs(currentRotation.minus(target).getDegrees());
                if (error < minError) {
                    minError = error;
                    bestAngle = target;
                }
            }
            Pose2d finalTargetPose = new Pose2d(
                    rawTargetPose.getTranslation(),
                    bestAngle);
            Logger.recordOutput("Superstructure/ToTrenchPose", finalTargetPose);
            return drive.driveHelper(finalTargetPose);

        }, Set.of(drive));
    }

    @Override
    public void periodic() {
    }
}
