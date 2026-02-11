// package frc.robot.subsystems.Shooter.Turret;

// import static edu.wpi.first.units.Units.Degree;
// import static edu.wpi.first.units.Units.DegreesPerSecond;
// import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.units.measure.Angle;
// import frc.robot.Constants.IDs;
// import frc.robot.subsystems.Shooter.ShooterConstants;
// import frc.robot.util.MathHelper.EncoderWithGearRatio;
// import frc.robot.util.MathHelper.RobustCRTCalculator;

// public class TurretIOTalon extends TurretIO {
//     private final TalonFX turretMotor;
//     private final CANcoder master, slave;

//     private final double gearRatio = 1.0;

//     private static final double HARD_MIN_RADS = ShooterConstants.HARD_MIN_RADS;
//     private static final double HARD_MAX_RADS = ShooterConstants.HARD_MAX_RADS;

//     private static final double SOFT_MIN_RADS = ShooterConstants.SOFT_MIN_RADS;
//     private static final double SOFT_MAX_RADS = ShooterConstants.SOFT_MAX_RADS;

    
//     private final MotionMagicVoltage m_request = new MotionMagicVoltage(Degree.of(0));

//     public TurretIOTalon() {
//         this.turretMotor = new TalonFX(IDs.Shooter.TURRET_MOTOR);
//         this.master = new CANcoder(IDs.Shooter.TURRET_MASTER_CANCODER);
//         this.slave = new CANcoder(IDs.Shooter.TURRET_SLAVE_CANCODER);
//         this.configureMotors();
//     }

//     public void configureMotors() {
//         TalonFXConfiguration configs = new TalonFXConfiguration();

//         // 電流限制
//         configs.CurrentLimits
//                 .withStatorCurrentLimitEnable(true)
//                 .withStatorCurrentLimit(70.0)
//                 .withSupplyCurrentLimitEnable(true)
//                 .withSupplyCurrentLimit(40.0);

//         configs.SoftwareLimitSwitch
//                 // 反向限位 (防止往下撞壞)
//                 .withReverseSoftLimitEnable(true)
//                 .withReverseSoftLimitThreshold(HARD_MIN_RADS)

//                 // 正向限位 (防止往上飛出去) - 強烈建議開啟保護硬體
//                 .withForwardSoftLimitEnable(true)
//                 .withForwardSoftLimitThreshold(HARD_MAX_RADS);

//         // 馬達設定
//         configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//         configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

//         // PID & FF
//         configs.Slot0.kS = 0.25;
//         configs.Slot0.kV = 0.12;
//         configs.Slot0.kA = 0.01;
//         configs.Slot0.kP = 0.11;
//         configs.Slot0.kI = 0.0;
//         configs.Slot0.kD = 0.0;

//         // Motion Magic
//         configs.MotionMagic.withMotionMagicCruiseVelocity(DegreesPerSecond.of(360))
//                 .withMotionMagicAcceleration(DegreesPerSecondPerSecond.of(720));

//         // 齒輪比
//         configs.Feedback.SensorToMechanismRatio = gearRatio;

//         turretMotor.getConfigurator().apply(configs);
//     }

//     @Override
//     public void setAngle(Rotation2d robotHeading, Angle targetRad, ShootState state) {
//         turretMotor.setControl(
//                 m_request.withPosition(Calculate(robotHeading, targetRad, state)));
//     }

//     @Override
//     public void resetAngle() {

//         EncoderWithGearRatio master = new EncoderWithGearRatio(
//                 this.master.getPosition().getValueAsDouble(),
//                 30);

//         EncoderWithGearRatio slave = new EncoderWithGearRatio(
//                 this.slave.getPosition().getValueAsDouble(),
//                 31);

//         double angle = RobustCRTCalculator.calculateAbsolutePosition(master, slave);
//         turretMotor.getConfigurator().setPosition(angle);
//     }

    
// }
