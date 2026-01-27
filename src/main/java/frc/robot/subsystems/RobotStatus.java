package frc.robot.subsystems;

import java.security.PublicKey;

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
    public boolean NeedResetPose = false;

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
    public boolean ifNeedResetPose(){
        this.NeedResetPose = this.drive.isClimbing();
        return NeedResetPose;
    }

    @Override
    public void periodic() {
        this.getArea();
        this.getVerticalSide();
        Logger.recordOutput("NeedResetPose", ifNeedResetPose());
    }
}
