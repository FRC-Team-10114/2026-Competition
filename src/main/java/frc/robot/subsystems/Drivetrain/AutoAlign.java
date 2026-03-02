package frc.robot.subsystems.Drivetrain;

import java.util.List;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IDs.Climber;
import frc.robot.commands.AutoChooser.AutoStart;
import frc.robot.commands.AutoChooser.IfGoclimb;
import frc.robot.commands.AutoChooser.ShowTime;
import frc.robot.util.FIeldHelper.AllianceFlipUtil;
import frc.robot.util.FIeldHelper.FieldTagMap;
import frc.robot.util.RobotStatus.RobotStatus;

public class AutoAlign {
    public final SendableChooser<Endclimb> EndClimbChooser = new SendableChooser<>();
    private final CommandSwerveDrivetrain drive;
    private final RobotStatus robotStatus;

    public AutoAlign(CommandSwerveDrivetrain drive, RobotStatus robotStatus) {
        this.drive = drive;
        this.robotStatus = robotStatus;
        chooser();

    }

    public void chooser() {
        EndClimbChooser.setDefaultOption("Climb", Endclimb.Climb);
        EndClimbChooser.addOption("ReseveClimb", Endclimb.ReseveClimb);
        EndClimbChooser.addOption("None", Endclimb.None);

        SmartDashboard.putData("End/Climb", EndClimbChooser);
    }
    // Pose Alignments Methods

    public enum Endclimb {
        None, Climb, ReseveClimb
    }

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

public Command FindClimbPath() {
        return Commands.defer(() -> {
            Endclimb EndClimb = EndClimbChooser.getSelected();
            PathPlannerPath climbPath = null;

            if (EndClimb == null) {
                return Commands.none();
            }

            try {
                switch (EndClimb) {
                    case Climb:
                        if (robotStatus.getVerticalSide() == RobotStatus.VerticalSide.TOP) {
                            climbPath = PathPlannerPath.fromChoreoTrajectory("End_Climb_Left");
                        } else {
                            climbPath = PathPlannerPath.fromChoreoTrajectory("End_Climb_Right");
                        }
                        break;
                    case ReseveClimb:
                        if (robotStatus.getVerticalSide() == RobotStatus.VerticalSide.TOP) {
                            climbPath = PathPlannerPath.fromChoreoTrajectory("End_Left_ReverseClimb");
                        } else {
                            climbPath = PathPlannerPath.fromChoreoTrajectory("Nnd_Right_ReverseClimb");
                        }
                        break;
                    default:
                        return Commands.none();
                }
            } catch (Exception e) {
                e.printStackTrace();
                return Commands.none();
            }

            if (climbPath != null) {
                PathConstraints constraints = new PathConstraints(
                        2.0, 2.5,
                        Units.degreesToRadians(540), Units.degreesToRadians(720));

                return AutoBuilder.pathfindThenFollowPath(
                        climbPath,
                        constraints);
            } else {
                return Commands.none();
            }
        }, Set.of(drive)); 
    }
}
