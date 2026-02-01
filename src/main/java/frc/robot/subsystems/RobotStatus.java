package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;

public class RobotStatus extends SubsystemBase {
    private static final double BLUE_ZONE_LIMIT = 5.50;

    private static final double RED_ZONE_START = 16.54 - 5.50; // 約 11.04

    private static final double FIELD_WIDTH = 8.21;
    private static final double MID_Y = FIELD_WIDTH / 2.0; // 中線 Y = 4.105
    public final CommandSwerveDrivetrain drive;

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

    public void updateOdometerStatus() {
        // 1. 從 Drive 取得現在是否傾斜 (假設你在 Drive 裡已經寫好了遲滯邏輯的 isClimbing)
        boolean isNowClimbing = this.drive.isClimbing();

        // 2. 下降邊緣偵測 (Falling Edge Detection)
        // 邏輯：上一幀還在爬 (True) + 這一幀變成平地 (False) = 剛落地！
        if (m_wasClimbing && !isNowClimbing) {
            
            // 觸發：舉起旗標！
            this.NeedResetPose = true;
            Logger.recordOutput("RobotStatus/Event", "Landed! Requesting Vision Reset");
        }

        // 3. 更新舊狀態
        m_wasClimbing = isNowClimbing;
    }

    @Override
    public void periodic() {
        this.getArea();
        this.getVerticalSide();
        this.updateOdometerStatus();
    }
}
