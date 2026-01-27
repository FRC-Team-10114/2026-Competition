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

import frc.robot.subsystems.Intake.IntakeSubssystem;

import frc.robot.subsystems.Shooter.ShooterCalculator;

import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.util.FieldTagMap;
import frc.robot.util.AllianceFlipUtil;

public class superstructure extends SubsystemBase {

    private final CommandSwerveDrivetrain drive;

    private final ShooterSubsystem shooterSubsystem;

    private final IntakeSubssystem intakeSubsystem;

    private static final double BLUE_ZONE_LIMIT = 5.50;

    private static final double RED_ZONE_START = 16.54 - 5.50; // 約 11.04

    private static final double FIELD_WIDTH = 8.21;
    private static final double MID_Y = FIELD_WIDTH / 2.0; // 中線 Y = 4.105

    public enum area {
        CENTER,
        BlueAlliance,
        RedAlliance
    }

    public enum VerticalSide {
        TOP,
        BOTTOM
    }

    public superstructure(
            CommandSwerveDrivetrain drive,
            ShooterSubsystem shooterSubsystem,
            IntakeSubssystem intakeSubssystem) {

        this.drive = drive;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubssystem;
    }

    public VerticalSide getVerticalSide() {
        double Y = drive.getPose2d().getY();
        if (Y > MID_Y) {
            return VerticalSide.TOP;
        } else {
            return VerticalSide.BOTTOM;
        }
    }

    public area getArea() {
        double x = drive.getPose2d().getX();

        if (x < BLUE_ZONE_LIMIT) {
            return area.BlueAlliance;
        } else if (x > RED_ZONE_START) {
            return area.RedAlliance;
        } else {
            return area.CENTER;
        }
    }

    public Pose2d ToTrenchPose() {

        Pose2d targetPose;
        if (this.getArea() == area.BlueAlliance || this.getArea() == area.RedAlliance) {
            if (this.getVerticalSide() == VerticalSide.TOP) {
                targetPose = FieldTagMap.getLeftTrenchPoses()[1];
            } else {
                targetPose = FieldTagMap.getRightTrenchPoses()[1];
            }

        Pose2d[] selectedTrench;
        if (this.getVerticalSide() == VerticalSide.TOP) {
            selectedTrench = FieldTagMap.getLeftTrenchPoses();

        } else {
            selectedTrench = FieldTagMap.getRightTrenchPoses();
        }
        Pose2d finalTarget;
        if(getarea() == area.CENTER){
            finalTarget = selectedTrench[0];       
        }else{
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
