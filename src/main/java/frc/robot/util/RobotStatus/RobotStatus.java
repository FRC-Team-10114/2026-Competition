package frc.robot.util.RobotStatus;

import java.util.ArrayList;
import java.util.EventObject;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;

public class RobotStatus extends SubsystemBase {

    public EventObject eventObject = new EventObject(getClass());

    private static final double BLUE_ZONE_LIMIT = 5.50;

    private static final double RED_ZONE_START = FieldConstants.fieldLength - 5.50; // 約 11.04
    private static final double MID_Y = FieldConstants.fieldWidth / 2.0; // 中線 Y = 4.105
    public final CommandSwerveDrivetrain drive;
    private final List<RototEvent> NeedResetPoseEvent = new ArrayList<>();

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

    public void addEvent(RototEvent event) {
        NeedResetPoseEvent.add(event);
    }

    public void updateOdometerStatus() {
        boolean isNowClimbing = this.drive.isClimbing();

        // 下降邊緣偵測 (剛落地)
        if (m_wasClimbing && !isNowClimbing) {

            // 2. [核心修改] 觸發事件：通知清單裡的所有人
            for (RototEvent listener : NeedResetPoseEvent) {
                listener.NeedResetPose();
            }
        }

        m_wasClimbing = isNowClimbing;
    }

    @Override
    public void periodic() {
        this.getArea();
        this.getVerticalSide();
        this.updateOdometerStatus();
    }
}