package frc.robot.commands;

import java.util.function.BooleanSupplier;

import org.opencv.ml.RTrees;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.RobotStatus.RobotStatus;

public class AutoChooser {
    private final CommandSwerveDrivetrain drive;
    private final superstructure superstructure;
    private final RobotStatus robotStatus;
    public final SendableChooser<ShowTime> ShowtimeChooser = new SendableChooser<>();
    public final SendableChooser<AutoStart> AutoStartChooser = new SendableChooser<>();
    public final SendableChooser<IfGoCenter> IfGoCenterChooser = new SendableChooser<>();
    public final SendableChooser<IfGoclimb> IfGoClimbChooser = new SendableChooser<>();

    public boolean isClimbEnabled = false;

    public BooleanSupplier ifclimb = () -> isClimbEnabled;

    public AutoChooser(CommandSwerveDrivetrain drive, superstructure superstructure, RobotStatus robotStatus) {
        this.drive = drive;
        this.superstructure = superstructure;
        this.robotStatus = robotStatus;

        this.configureAutoChoosers();
        this.SetNamedCommands();
    }

    public void SetNamedCommands() {
        NamedCommands.registerCommand("intakeDown", superstructure.intake());
        NamedCommands.registerCommand("intakestop", superstructure.stopintake());
        NamedCommands.registerCommand("HoodDown", Commands.runOnce(() -> robotStatus.SetSafeHood(true), robotStatus));
        NamedCommands.registerCommand("StopHoodDown",
                Commands.runOnce(() -> robotStatus.SetSafeHood(false), robotStatus));
        NamedCommands.registerCommand("shoot", superstructure.autoshooter().withTimeout(2.0));
        NamedCommands.registerCommand("NoStopShoot", superstructure.autoshooter());
        NamedCommands.registerCommand("stopshoot", superstructure.stopShoot());
        NamedCommands.registerCommand("isIn",
                Commands.run(() -> System.out.println("Stop!!!!!!!")).until(robotStatus::isInTrench));
        NamedCommands.registerCommand("ClimbPrepare", superstructure.ClimbPrepare());
        NamedCommands.registerCommand("Climb", superstructure.Climb());
        NamedCommands.registerCommand("Climbdown", superstructure.Climb());

    }

    public Command ClimbPrepare() {
        return Commands.either(superstructure.ClimbPrepare(), Commands.none(), ifclimb);
    }

    public enum ShowTime {
        None, LeftDoubleCenter, RightDoubleCenter, LeftCleanAllCenter, RightCleanAllCenter
    }

    public enum AutoStart {
        LEFT, CENTER, RIGHT, NONE
    }

    public enum IfGoCenter {
        GO_CENTER,
        stay
    }

    public enum IfGoclimb {
        climb,
        ReverseClimb,
        EndAtCenter,
        None
    }

    public void configureAutoChoosers() {

        AutoStartChooser.setDefaultOption("None", AutoStart.NONE);
        AutoStartChooser.addOption("Start: Left", AutoStart.LEFT);
        AutoStartChooser.addOption("Start: Right", AutoStart.RIGHT);
        AutoStartChooser.addOption("Start: CENTER", AutoStart.CENTER);

        ShowtimeChooser.setDefaultOption("None", ShowTime.None);
        ShowtimeChooser.addOption("LeftDoubleCenter", ShowTime.LeftDoubleCenter);
        ShowtimeChooser.addOption("LeftCleanAllCenter", ShowTime.LeftCleanAllCenter);
        ShowtimeChooser.addOption("RightCleanAllCenter", ShowTime.RightCleanAllCenter);

        IfGoCenterChooser.setDefaultOption("Go Center", IfGoCenter.GO_CENTER);
        IfGoCenterChooser.addOption("Stay", IfGoCenter.stay);

        IfGoClimbChooser.setDefaultOption("End At Center", IfGoclimb.EndAtCenter);
        IfGoClimbChooser.addOption("Reverse Climb", IfGoclimb.ReverseClimb);
        IfGoClimbChooser.addOption("Climb", IfGoclimb.climb);
        IfGoClimbChooser.addOption("None", IfGoclimb.None);

        SmartDashboard.putData("Auto/End", IfGoClimbChooser);
        SmartDashboard.putData("Auto/If Go Center", IfGoCenterChooser);
        SmartDashboard.putData("Auto/Start Position", AutoStartChooser);
        SmartDashboard.putData("Auto/If show time", ShowtimeChooser);
    }

    public Command auto() {
        AutoStart startPose = AutoStartChooser.getSelected();
        ShowTime issShowTime = ShowtimeChooser.getSelected();
        IfGoCenter ifGoCenter = IfGoCenterChooser.getSelected();
        IfGoclimb ifGoclimb = IfGoClimbChooser.getSelected();

        // 1. 增加 Null 防護，避免儀表板未同步導致 Crash
        if (startPose == null)
            startPose = AutoStart.NONE;
        if (issShowTime == null)
            issShowTime = ShowTime.None;
        if (ifGoCenter == null)
            ifGoCenter = IfGoCenter.GO_CENTER;
        if (ifGoclimb == null)
            ifGoclimb = IfGoclimb.None;

        if (startPose == AutoStart.NONE) {

            Pose2d currentPose = this.drive.getPose2d();

            try {
                Pose2d leftStart = PathPlannerPath.fromChoreoTrajectory("Left_center_deploy")
                        .getStartingHolonomicPose()
                        .orElse(new Pose2d());

                Pose2d centerStart = PathPlannerPath.fromChoreoTrajectory("Center")
                        .getStartingHolonomicPose()
                        .orElse(new Pose2d());

                Pose2d rightStart = PathPlannerPath.fromChoreoTrajectory("Right_center_hum")
                        .getStartingHolonomicPose()
                        .orElse(new Pose2d());

                // C. 計算距離 (使用 getTranslation().getDistance())
                double distLeft = currentPose.getTranslation().getDistance(leftStart.getTranslation());
                double distCenter = currentPose.getTranslation().getDistance(centerStart.getTranslation());
                double distRight = currentPose.getTranslation().getDistance(rightStart.getTranslation());

                // D. 比較誰最近
                if (distLeft < distCenter && distLeft < distRight) {
                    startPose = AutoStart.LEFT;
                } else if (distRight < distCenter && distRight < distLeft) {
                    startPose = AutoStart.RIGHT;
                } else {
                    startPose = AutoStart.CENTER;
                }

            } catch (Exception e) {
                e.printStackTrace();
                startPose = AutoStart.RIGHT;
            }
        }

        Command autoCommand = null;

        switch (issShowTime) {
            case LeftDoubleCenter:
                return new PathPlannerAuto("Left_DoubleCenter");
            case RightDoubleCenter:
                return new PathPlannerAuto("Right_DoubleCenter");
            case LeftCleanAllCenter:
                return new PathPlannerAuto("Left_CleanAllCenter");
            case RightCleanAllCenter:
                return new PathPlannerAuto("Right_CleanAllCenter");
            default:
                break;
        }

        // --- 組合路徑邏輯 ---
        Command start = Commands.none();
        switch (startPose) {
            case LEFT:
                start = (ifGoCenter == IfGoCenter.GO_CENTER) ? new PathPlannerAuto("Left_center_deploy")
                        : new PathPlannerAuto("Left_Deploy");
                break;
            case RIGHT:
                start = (ifGoCenter == IfGoCenter.GO_CENTER) ? new PathPlannerAuto("Right_center_hum")
                        : new PathPlannerAuto("Right_hum");
                break;
            case CENTER:
                start = new PathPlannerAuto("Center");
                break;
            default:
                break;
        }

        Command end = Commands.none();
        switch (ifGoclimb) {
            case climb:
                if (startPose == AutoStart.LEFT)
                    end = new PathPlannerAuto("Left_End_Climb");
                else if (startPose == AutoStart.RIGHT)
                    end = new PathPlannerAuto("Right_End_Climb");
                else if (startPose == AutoStart.CENTER)
                    end = new PathPlannerAuto("Center_End_Climb");
                isClimbEnabled = true;
                break;
            case ReverseClimb:
                if (startPose == AutoStart.LEFT)
                    end = new PathPlannerAuto("Left_End_ReverseClimb");
                else if (startPose == AutoStart.RIGHT)
                    end = new PathPlannerAuto("Right_End_ReverseClimb");
                else if (startPose == AutoStart.CENTER)
                    end = new PathPlannerAuto("Center_End_ReverseClimb");
                isClimbEnabled = true;
                break;
            case EndAtCenter:
                if (startPose == AutoStart.LEFT)
                    end = new PathPlannerAuto("Left_End_Center");
                else if (startPose == AutoStart.RIGHT)
                    end = new PathPlannerAuto("Right_End_Center");
                // 可在此補上 CENTER 起點的邏輯 (如果有)
                break;
            default:
                break;
        }

        return Commands.sequence(start, end);
    }
    // public Command auto_down(){
    //     IfGoclimb ifGoclimb = IfGoClimbChooser.getSelected();
    // }
}
