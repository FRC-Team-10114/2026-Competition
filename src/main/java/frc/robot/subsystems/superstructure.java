package frc.robot.subsystems;

import java.util.List;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.RoundOne;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.util.FieldTagMap;
import frc.robot.util.AllianceFlipUtil;

public class superstructure extends SubsystemBase {

    private final CommandSwerveDrivetrain drive;

    private final IntakeSubsystem intake;

    private static final double BLUE_ZONE_LIMIT = 5.50;

    private static final double RED_ZONE_START = 16.54 - 5.50; // 約 11.04

    private static final double FIELD_WIDTH = 8.21;
    private static final double MID_Y = FIELD_WIDTH / 2.0; // 中線 Y = 4.105

    private final ShooterSubsystem shooterSubsystem;

    public enum area {
        CENTER,
        BlueAlliance,
        RedAlliance
    }

    public enum VerticalSide {
        TOP,
        BOTTOM
    }

    public superstructure(IntakeSubsystem intake, CommandSwerveDrivetrain drive, ShooterSubsystem shooterSubsystem) {
        this.intake = intake;
        this.drive = drive;
        this.shooterSubsystem = shooterSubsystem;
    }

        public VerticalSide getVerticalSide() {
        double Y = drive.getPose2d().getY();
        if (Y > MID_Y) {
            return VerticalSide.TOP;
        } else {
            return VerticalSide.BOTTOM;
        }
    }

    public area getarea() {
        double x = drive.getPose2d().getX();

        if (x < BLUE_ZONE_LIMIT) {
            return area.BlueAlliance;
        } else if (x > RED_ZONE_START) {
            return area.RedAlliance;
        } else {
            return area.CENTER;
        }
    }

    public Command intake() {
        return Commands.runOnce(() -> this.intake.intake(), this.intake);
    }

    public Command outtake() {
        return Commands.runOnce(() -> this.intake.outtake(), this.intake);
    }

    public Command stopIntakeMotor() {
        return Commands.runOnce(() -> this.stopIntakeMotor(), this.intake);
    }

    public Pose2d ToTrenchPose() {
        Pose2d targetPose;
        if (this.getarea() == area.BlueAlliance || this.getarea() == area.RedAlliance) {
            if (this.getVerticalSide() == VerticalSide.TOP) {
                targetPose = FieldTagMap.getLeftTrenchPoses()[1];
            } else {
                targetPose = FieldTagMap.getRightTrenchPoses()[1];
            }
        } else {
            if (this.getVerticalSide() == VerticalSide.TOP) {
                targetPose = FieldTagMap.getLeftTrenchPoses()[0];
            } else {
                targetPose = FieldTagMap.getRightTrenchPoses()[0];
            }
        }
        return AllianceFlipUtil.apply(targetPose);
    }

    public Command DriveToTrench() {
        List<Rotation2d> snapAngles = List.of(
                Rotation2d.fromDegrees(0),
                Rotation2d.fromDegrees(45),
                Rotation2d.fromDegrees(90),
                Rotation2d.fromDegrees(135),
                Rotation2d.fromDegrees(180),
                Rotation2d.fromDegrees(-45),
                Rotation2d.fromDegrees(-90),
                Rotation2d.fromDegrees(-135));

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
            return drive.driveHelper(finalTargetPose);

        }, Set.of(drive));
    }

    @Override
    public void periodic() {
        Logger.recordOutput("ToTrenchPose", this.ToTrenchPose());
        this.shooterSubsystem.setTurretAngle(drive.getRotation(), null);
    }
}
