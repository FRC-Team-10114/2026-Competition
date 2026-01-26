package frc.robot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public class Constants {
    private static final AprilTagFieldLayout layout;
    public static final double fieldLength;
    public static final double fieldWidth;
    static {
        try {
            // 載入預設場地 (例如 2025 Reefscape 或 2026)
            layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        } catch (Exception e) {
            throw new RuntimeException("地圖載入失敗", e);
        }
    }
    static {
        AprilTagFieldLayout layout;
        try {
            // 自動載入當年度的預設場地 (例如 2026 場地)
            layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        } catch (Exception e) {
            // 萬一讀不到檔案 (極少發生)，給個預設值防止程式崩潰
            // 這裡可以填入規則書上的大約數值
            layout = null;
            e.printStackTrace();
        }

        if (layout != null) {
            // 從官方資料直接抓取精確數值
            fieldLength = layout.getFieldLength();
            fieldWidth = layout.getFieldWidth();
        } else {
            // Fallback (保底數值)
            fieldLength = 16.54;
            fieldWidth = 8.21;
        }
    }

    public static final class SwerveModuleConstants {
        public static final String[] ModuleName = {
                "ForntLeft",
                "FrontRight",
                "BackLeft",
                "BackRight"
        };
    }

    public static final class IntakeConstants {

        public static final int INTAKE_ARM_MOTOR_ID = 13;
        public static final int INTKAE_ARM_CANCODER_ID = 15;

        public static final double INTAKE_ARM_GEAR_RATIO = 0.0;
        public static final double INTAKE_ARM_VELOCITY_CONVERSION_FACOTR = RadiansPerSecond
                .convertFrom(INTAKE_ARM_GEAR_RATIO, RotationsPerSecond);
        public static final double INTAKE_ARM_POSITION_CONVERSION_FACTOR = Radians.convertFrom(INTAKE_ARM_GEAR_RATIO,
                Rotation);

        public static final double[] INTAKE_ARM_PID = { 0.1, 0.0, 0.0 };
        public static final double INTAKE_ARM_CRUISE_VELOCITY = RadiansPerSecond.fromBaseUnits(10);
        public static final double INTAKE_ARM_MAX_ACCELERATION = RadiansPerSecondPerSecond.fromBaseUnits(20);

        public static final Voltage INTAKE_ARM_STATOR_CURRENT_LIMIT = Volts.of(50);
        public static final Voltage INTAKE_ARM_SUPPLY_CURRENT_LIMIT = Volts.of(40);

        public static final Angle INTAKE_ARM_FORWARD_LIMIT = Radians.of(0.0);
        public static final Angle INTAKE_ARM_REVERSE_LIMIT = Radians.of(0.0);

        public static final int INTAKE_ROLLER_MOTOR_ID = 14;

        public static final double[] INTAKE_ROLLER_PID = { 0.1, 0.0, 0.0 };

        public static final double INTAKE_ROLLER_CRUISE_VELOCITY = RadiansPerSecond.fromBaseUnits(10);
        public static final double INTAKE_ROLLER_MAX_ACCELERATION = RadiansPerSecondPerSecond.fromBaseUnits(20);

        public static final double INTAKE_ROLLER_GEAR_RATIO = 0.0;
        public static final double INTAKE_ROLLER_VELOCITY_CONVERSION_FACOTR = RotationsPerSecond
                .convertFrom(INTAKE_ROLLER_GEAR_RATIO, RotationsPerSecond);
        public static final double INTAKE_ROLLER_POSITION_CONVERSION_FACTOR = Rotation.convertFrom(
                INTAKE_ROLLER_GEAR_RATIO,
                Rotation);

        public static final Voltage INTAKE_ROLLER_STATOR_CURRENT_LIMIT = Volts.of(50);
        public static final Voltage INTAKE_ROLLER_SUPPLY_CURRENT_LIMIT = Volts.of(40);
    }

    public static final class LimelightConstants {
        public static final double MAX_GYRO_RATE = 1080;
    }

    public static final class HoodConstants {
        private final double GEAR_RATIO = 1;
        private final double POSITION_CONVERSION_FACTOR = (2 * Math.PI) / GEAR_RATIO;
    }

    public static final class PhotonVisionConstants {

        public record CameraConfig(String cameraName, Transform3d cameraLocation) {
        }

        public static final CameraConfig newCam = new CameraConfig(
                "NewCam",
                new Transform3d(
                        new Translation3d(0.0, 0.0, 0.0),
                        new Rotation3d(0, 0, 0)));

        public static final double borderPixels = 15.0; // 拒絕貼邊緣的角點（避免畸變/遮擋）
        public static final double maxSingleTagDistanceMeters = Units.feetToMeters(6.0); // 單tag最遠可接受距離
        public static final double maxYawRate = 720.0;// 最大可以接受的旋轉速度
    }

    public class siteConstants {
        // Dimensions
        public static final double width = Units.inchesToMeters(31.8);
        public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
        public static final double height = Units.inchesToMeters(7.0);

        public static final Translation3d topCenterPoint = new Translation3d(
                layout.getTagPose(26).get().getX() + width / 2.0, // 取得 Tag 26 的 X 並加上半寬
                fieldWidth / 2.0, // Y 軸置中
                height // 高度固定
        );
    }
}