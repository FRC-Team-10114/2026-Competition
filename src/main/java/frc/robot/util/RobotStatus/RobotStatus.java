package frc.robot.util.RobotStatus;

import java.util.ArrayList;
import java.util.EventObject;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.siteConstants;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.FIeldHelper.AllianceFlipUtil;
import frc.robot.util.RobotEvent.Event.*;

public class RobotStatus extends SubsystemBase {

    public EventObject eventObject = new EventObject(getClass());

    private boolean xx = false;

    private static final double BLUE_ZONE_LIMIT = 5.50;
    private static final double RED_ZONE_START = FieldConstants.fieldLength - 5.50; // 約 11.04
    private static final double MID_Y = FieldConstants.fieldWidth / 2.0; // 中線 Y = 4.105

    public final CommandSwerveDrivetrain drive;

    // 將變數名稱改為小寫開頭 (Java 慣例)，避免跟類別名稱混淆
    private final List<NeedResetPoseEvent> needResetPoseEvents = new ArrayList<>();
    private final List<InTrench> inTrenchEvents = new ArrayList<>();
    private final List<NotInTrench> notInTrenchEvents = new ArrayList<>();

    public enum Area {
        CENTER,
        BlueAlliance,
        RedAlliance
    }

    public enum VerticalSide {
        TOP,
        BOTTOM
    }

    // 這個公開變數就是我們的 "旗標"
    public boolean NeedResetPose = false;

    // 用來記住 "上一次" 是否在爬坡 (為了偵測下降邊緣)
    private boolean m_wasClimbing = false;

    public RobotStatus(CommandSwerveDrivetrain drive) {
        this.drive = drive;
    }

    public VerticalSide getVerticalSide() {
        double Y = drive.getPose2d().getY();
        if (Y > MID_Y) {
            return VerticalSide.TOP;
        } else {
            return VerticalSide.BOTTOM;
        }
    }

    public Area getArea() {
        double x = drive.getPose2d().getX();

        if (x < BLUE_ZONE_LIMIT) {
            return Area.BlueAlliance;
        } else if (x > RED_ZONE_START) {
            return Area.RedAlliance;
        } else {
            return Area.CENTER;
        }
    }

    public void TriggerNeedResetPoseEvent(NeedResetPoseEvent event) {
        needResetPoseEvents.add(event);
    }

    public void TriggerInTrench(InTrench event) {
        inTrenchEvents.add(event);
    }

    public void TriggerNotInTrench(NotInTrench event) {
        notInTrenchEvents.add(event);
    }

    public void updateOdometerStatus() {
        // 假設 drive 有 isClimbing() 方法。如果沒有，請確認你的 Drivetrain 程式碼
        boolean isNowClimbing = this.drive.isClimbing();

        // 下降邊緣偵測 (剛落地)
        if (m_wasClimbing && !isNowClimbing) {
            // 觸發事件：通知清單裡的所有人
            for (NeedResetPoseEvent listener : needResetPoseEvents) {
                listener.NeedResetPose();
            }
        }

        m_wasClimbing = isNowClimbing;
    }

    // *** 關鍵修復 ***
    // 這裡修復了 Crash: java.util.NoSuchElementException
    public boolean isInMyAllianceZone() {
        // 1. 取得目前 FMS 的聯盟顏色
        Optional<Alliance> ally = DriverStation.getAlliance();

        // 2. 安全檢查：如果還沒連上 FMS/DriverStation (例如模擬剛啟動)，它是空的
        // 直接回傳 false 避免崩潰
        if (ally.isEmpty()) {
            return false;
        }

        // 3. 取得機器人目前在哪個區域
        Area currentArea = getArea();

        // 4. 進行比對 (現在確認有值了，才可以用 .get())
        if (ally.get() == Alliance.Blue) {
            // 我是藍隊，我在藍區嗎？
            return currentArea == Area.BlueAlliance;
        } else {
            // 我是紅隊，我在紅區嗎？
            return currentArea == Area.RedAlliance;
        }
    }

    public void isInTrenchEvent() {
        if (isInTrench()) {
            for (InTrench listener : inTrenchEvents) {
                listener.InTrench();
            }
        } else {
            for (NotInTrench listener : notInTrenchEvents) {
                listener.NotInTrench();
            }
        }
    }

    public void xtrue(){
        this.xx = true;
    }   

    public void xfalse() {
        this.xx = false;
    }

    public boolean isInTrench() {
        var currentPose = drive.getPose2d();

        // 檢查各個 Trench 區域
        boolean inRight = isInside(currentPose,
                siteConstants.Right_TRENCHE_Pose1,
                siteConstants.Right_TRENCHE_Pose2,
                siteConstants.Right_TRENCHE_Pose3);

        boolean inLeft = isInside(currentPose,
                siteConstants.Left_TRENCHE_Pose1,
                siteConstants.Left_TRENCHE_Pose2,
                siteConstants.Left_TRENCHE_Pose3);

        boolean inRightFlipped = isInside(currentPose,
                AllianceFlipUtil.Needapply(siteConstants.Right_TRENCHE_Pose1),
                AllianceFlipUtil.Needapply(siteConstants.Right_TRENCHE_Pose2),
                AllianceFlipUtil.Needapply(siteConstants.Right_TRENCHE_Pose3));

        boolean inLeftFlipped = isInside(currentPose,
                AllianceFlipUtil.Needapply(siteConstants.Left_TRENCHE_Pose1),
                AllianceFlipUtil.Needapply(siteConstants.Left_TRENCHE_Pose2),
                AllianceFlipUtil.Needapply(siteConstants.Left_TRENCHE_Pose3));

        return inRight || inLeft || inRightFlipped || inLeftFlipped || xx;
    }

    private boolean isInside(Pose2d robotPose, Pose2d... corners) {
        // 安全檢查，避免空指針錯誤
        if (corners == null || corners.length == 0) return false;

        double x = robotPose.getX();
        double y = robotPose.getY();

        // 初始化極大與極小值
        double minX = Double.MAX_VALUE;
        double maxX = Double.MIN_VALUE;
        double minY = Double.MAX_VALUE;
        double maxY = Double.MIN_VALUE;

        // 自動找出所有角落中的最大與最小範圍
        for (Pose2d corner : corners) {
            if (corner != null) { // 確保 corner 不是 null
                minX = Math.min(minX, corner.getX());
                maxX = Math.max(maxX, corner.getX());
                minY = Math.min(minY, corner.getY());
                maxY = Math.max(maxY, corner.getY());
            }
        }
        return (x >= minX && x <= maxX) && (y >= minY && y <= maxY);
    }

    @Override
    public void periodic() {
        // 更新狀態
        this.updateOdometerStatus();
        this.isInTrenchEvent();

        // 將重要數據記錄到 Log (AdvantageScope / Shuffleboard)
        Logger.recordOutput("RobotStatus/InMyAllianceZone", isInMyAllianceZone());
        Logger.recordOutput("RobotStatus/Area", getArea().toString());
        Logger.recordOutput("RobotStatus/VerticalSide", getVerticalSide().toString());
        Logger.recordOutput("RobotStatus/xx", xx);
    }
}