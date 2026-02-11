package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;

public class Auto {
    private final CommandSwerveDrivetrain drive;

    public Auto(CommandSwerveDrivetrain drive) {
        this.drive = drive;
        this.configureAutoChoosers();
    }

    public enum AutoStart {
        LEFT, CENTER, RIGHT, NONE
    }

    public enum RoundOne {
        GO_CENTER,
        GO_DEPOT,
        GO_OUTPOST,
        GO_TOWER,
        NONE
    }

    public enum RoundTwo {
        GO_CENTER,
        GO_DEPOT,
        GO_OUTPOST,
        GO_TOWER,
        NONE
    }

    public final SendableChooser<AutoStart> AutoStartChooser = new SendableChooser<>();
    public final SendableChooser<RoundOne> AutoRoundOneChooser = new SendableChooser<>();
    public final SendableChooser<RoundTwo> AutoRoundTwoChooser = new SendableChooser<>();

    public void configureAutoChoosers() {

        AutoStartChooser.setDefaultOption("None", AutoStart.NONE);
        AutoStartChooser.addOption("Start: Left", AutoStart.LEFT);
        AutoStartChooser.addOption("Start: Right", AutoStart.RIGHT);
        AutoStartChooser.addOption("Start: CENTER", AutoStart.CENTER);
        SmartDashboard.putData("Auto/1. Start Position", AutoStartChooser);

        AutoRoundOneChooser.setDefaultOption("R1: Go Center", RoundOne.GO_CENTER);
        AutoRoundOneChooser.addOption("R1: Go Center", RoundOne.GO_CENTER);
        AutoRoundOneChooser.addOption("R1: Go Depot", RoundOne.GO_DEPOT);
        AutoRoundOneChooser.addOption("R1: Go Outpost", RoundOne.GO_OUTPOST);
        AutoRoundOneChooser.addOption("R1: Go Tower", RoundOne.GO_TOWER);
        AutoRoundOneChooser.addOption("R1: NONE", RoundOne.NONE);
        SmartDashboard.putData("Auto/2. Round One", AutoRoundOneChooser);

        AutoRoundTwoChooser.setDefaultOption("R2: Go Center", RoundTwo.GO_CENTER);
        AutoRoundTwoChooser.addOption("R2: Go Center", RoundTwo.GO_CENTER);
        AutoRoundTwoChooser.addOption("R2: Go Depot", RoundTwo.GO_DEPOT);
        AutoRoundTwoChooser.addOption("R2: Go Outpost", RoundTwo.GO_OUTPOST);
        AutoRoundTwoChooser.addOption("R2: Go Tower", RoundTwo.GO_TOWER);
        AutoRoundTwoChooser.addOption("R2: Go NONE", RoundTwo.NONE);
        SmartDashboard.putData("Auto/2. Round Two", AutoRoundTwoChooser);
    }

    public Command auto() {
        // 1. 取得選項
        AutoStart startPose = AutoStartChooser.getSelected();
        RoundOne roundOneDo = AutoRoundOneChooser.getSelected();
        RoundTwo roundTwoDo = AutoRoundTwoChooser.getSelected();

        // 2. 決定起始路徑
        Command start = null;

        if (start == null) {

            Pose2d currentPose = this.drive.getPose2d();

            try {
                Pose2d leftStart = PathPlannerPath.fromChoreoTrajectory("Left_start")
                        .getStartingHolonomicPose()
                        .orElse(new Pose2d());

                Pose2d centerStart = PathPlannerPath.fromChoreoTrajectory("Center_start")
                        .getStartingHolonomicPose()
                        .orElse(new Pose2d());

                Pose2d rightStart = PathPlannerPath.fromChoreoTrajectory("Right_start")
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
                startPose = AutoStart.LEFT;
            }
        }
        switch (startPose) {
            case LEFT:
                start = new PathPlannerAuto("Left_start");
                break;
            case CENTER:
                start = new PathPlannerAuto("Center_start");
                break;
            case RIGHT:
                start = new PathPlannerAuto("Right_start");
                break;
        }

        // 3. 決定第一輪任務
        Command RoundOne = Commands.none();
        switch (startPose) {
            case LEFT:
                switch (roundOneDo) {
                    case GO_CENTER:
                        RoundOne = new PathPlannerAuto("Left_Center_Left");
                        break;
                    case GO_DEPOT:
                        RoundOne = new PathPlannerAuto("Left_DEPOT_LEFT");
                        break;
                    case GO_OUTPOST:
                        RoundOne = new PathPlannerAuto("Left_DEPOST_Left");
                        break;
                    case GO_TOWER:
                        RoundOne = new PathPlannerAuto("Left_Tower");
                        break;
                    case NONE:
                        break;
                }
                break; // ✅ 這裡一定要加 break，不然會跑去執行 RIGHT 的邏輯

            case RIGHT:
                switch (roundOneDo) {
                    case GO_CENTER:
                        RoundOne = new PathPlannerAuto("Right_Center_Right");
                        break;
                    case GO_DEPOT:
                        RoundOne = new PathPlannerAuto("Right_DEPOT_Right");
                        break;
                    case GO_OUTPOST:
                        RoundOne = new PathPlannerAuto("Right_DEPOST_Right");
                        break;
                    case GO_TOWER:
                        RoundOne = new PathPlannerAuto("Right_Tower");
                        break;
                    case NONE:
                        break;
                }
                break; // ✅ 補上 break

            case CENTER:
                switch (roundOneDo) {
                    case GO_CENTER:
                        RoundOne = new PathPlannerAuto("Center_Center_Center");
                        break;
                    case GO_DEPOT:
                        RoundOne = new PathPlannerAuto("Center_DEPOT_Center");
                        break;
                    case GO_OUTPOST:
                        RoundOne = new PathPlannerAuto("Center_DEPOST_Center");
                        break;
                    case GO_TOWER:
                        RoundOne = new PathPlannerAuto("Center_Tower");
                        break;
                    case NONE:
                        break;
                }
                break; // ✅ 補上 break
        }

        // 4. 決定第二輪任務 (修正變數指派錯誤)
        Command RoundTwo = Commands.none();
        switch (startPose) {
            case LEFT:
                switch (roundTwoDo) {
                    case GO_CENTER:
                        RoundTwo = new PathPlannerAuto("Left_Center_Left");
                        break;
                    case GO_DEPOT:
                        RoundTwo = new PathPlannerAuto("Left_DEPOT_LEFT");
                        break;
                    case GO_OUTPOST:
                        RoundTwo = new PathPlannerAuto("Left_DEPOST_Left");
                        break;
                    case GO_TOWER:
                        RoundTwo = new PathPlannerAuto("Left_Tower");
                        break;
                    case NONE:
                        break;
                }
                break; // ✅ 補上 break

            case RIGHT:
                switch (roundTwoDo) {
                    case GO_CENTER:
                        RoundTwo = new PathPlannerAuto("Right_Center_Right");
                        break;
                    case GO_DEPOT:
                        RoundTwo = new PathPlannerAuto("Right_DEPOT_Right");
                        break;
                    case GO_OUTPOST:
                        RoundTwo = new PathPlannerAuto("Right_DEPOST_Right");
                        break;
                    case GO_TOWER:
                        RoundTwo = new PathPlannerAuto("Right_Tower");
                        break;
                    case NONE:
                        break;
                }
                break; // ✅ 補上 break

            case CENTER:
                switch (roundTwoDo) {
                    case GO_CENTER:
                        RoundTwo = new PathPlannerAuto("Center_Center_Center");
                        break;
                    case GO_DEPOT:
                        RoundTwo = new PathPlannerAuto("Center_DEPOT_Center");
                        break;
                    case GO_OUTPOST:
                        RoundTwo = new PathPlannerAuto("Center_DEPOST_Center");
                        break;
                    case GO_TOWER:
                        RoundTwo = new PathPlannerAuto("Center_Tower");
                        break;
                    case NONE:
                        break;
                }
                break; // ✅ 補上 break
        }

        // 5. 串聯執行
        return Commands.sequence(
                start,
                RoundOne,
                RoundTwo);
    }

    public void startResetPose() {
        AutoStart selectedStart = AutoStartChooser.getSelected();
        String pathName = null;

        switch (selectedStart) {
            case LEFT:
                pathName = "Left_start";
                break;
            case CENTER:
                pathName = "Center_start";
                break;
            case RIGHT:
                pathName = "Right_start";
                break;
            case NONE:
            default:
                System.out.println("[Auto] No Start Position Selected, skipping reset.");
                return;
        }

        // 3. 讀取路徑並重置座標
        if (pathName != null) {
            try {
                // 讀取 PathPlanner 的路徑檔案
                // 注意：如果你是用 .path 檔，請用 PathPlannerPath.fromPathFile(pathName)
                // 如果你是用 Choreo .traj 檔，請維持你原本用的 fromChoreoTrajectory
                PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(pathName);

                // 取得該路徑的 "起點座標" (Starting Pose)
                // orElse(new Pose2d()) 是防止讀不到時程式崩潰
                Pose2d startPose = path.getStartingHolonomicPose().orElse(new Pose2d());

                // 強制重置 Drive 的里程計
                drive.resetPose(startPose);

            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}