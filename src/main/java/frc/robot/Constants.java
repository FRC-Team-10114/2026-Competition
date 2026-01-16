package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final class SwerveModuleConstants {
        public static final String[] ModuleName = {
                "ForntLeft",
                "FrontRight",
                "BackLeft",
                "BackRight"
        };
    }

    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 13;
    }

    public static final class LimelightConstants {
        public static final double MAX_GYRO_RATE = 1080;
    }

    public static final class HoodConstants {
        private final double GEAR_RATIO = 1;
        private final double POSITION_CONVERSION_FACTOR = (2 * Math.PI) / GEAR_RATIO;
    }

    public static final class PhotonVisionConstants {

        public record CameraConfig(String cameraName, Transform3d cameraLocation) {}

        public static final CameraConfig newCam = new CameraConfig(
                "NewCam", 
                new Transform3d(
                        new Translation3d(0.0, 0.0, 0.0), 
                        new Rotation3d(0, 0, 0)));

        public static final double borderPixels = 15.0; // 拒絕貼邊緣的角點（避免畸變/遮擋）
        public static final double maxSingleTagDistanceMeters = Units.feetToMeters(6.0); // 單tag最遠可接受距離
        public static final double maxYawRate = 720.0;// 最大可以接受的旋轉速度
    }
}
