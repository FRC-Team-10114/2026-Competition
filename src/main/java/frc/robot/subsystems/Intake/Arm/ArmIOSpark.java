// package frc.robot.subsystems.Intake.Arm;

// import static edu.wpi.first.units.Units.Radian;
// import static edu.wpi.first.units.Units.Rotation;

// import com.ctre.phoenix6.hardware.CANcoder;
// import com.revrobotics.PersistMode;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.ResetMode;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.ControlType;

// import edu.wpi.first.units.measure.Angle;
// import frc.robot.Constants.IDs;
// import frc.robot.subsystems.Intake.IntakeConstants.ArmConstants;

// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// public class ArmIOSpark implements ArmIO {

//     private final SparkMax armMotor;

//     private final RelativeEncoder armEncoder;

//     private final SparkClosedLoopController armContoller;

//     private final CANcoder armAbsoluteEncoder;

//     public ArmIOSpark() {

//         this.armMotor = new SparkMax(IDs.Intake.ARM_MOTOR, MotorType.kBrushless);
//         this.armEncoder = armMotor.getEncoder();
//         this.armContoller = armMotor.getClosedLoopController();

//         this.armAbsoluteEncoder = new CANcoder(IDs.Intake.ARM_CANCODER);

//         configure();
//         resetEncoder();
//     }

//     @Override
//     public void setPosition(Angle position) {
//         this.armContoller.setSetpoint(position.baseUnitMagnitude(), ControlType.kMAXMotionPositionControl);
//     }

//     @Override
//     public Angle getPosition() {
//         return Radian.ofBaseUnits(this.armEncoder.getPosition());
//     }

//     @Override
//     public void resetEncoder() {
//         this.armEncoder
//                 .setPosition(Radian.convertFrom(this.armAbsoluteEncoder.getPosition().getValueAsDouble(), Rotation));
//     }

//     @Override
//     public void configure() {

//         var armConfig = new SparkMaxConfig();

//         armConfig
//                 .idleMode(IdleMode.kBrake)
//                 .inverted(false)
//                 .smartCurrentLimit((int) ArmConstants.SUPPLY_CURRENT_LIMIT.baseUnitMagnitude())
//                 .apply(armConfig);
//         armConfig.encoder
//                 .positionConversionFactor(ArmConstants.POSITION_CONVERSION_FACTOR)
//                 .apply(armConfig.encoder);
//         armConfig.closedLoop
//                 .pid(
//                         ArmConstants.PID[0],
//                         ArmConstants.PID[1],
//                         ArmConstants.PID[2])
//                 .apply(armConfig.closedLoop);
//         armConfig.closedLoop.maxMotion
//                 .cruiseVelocity(ArmConstants.CRUISE_VELOCITY)
//                 .maxAcceleration(ArmConstants.MAX_ACCELERATION)
//                 .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
//                 .apply(armConfig.closedLoop.maxMotion);
//         armConfig.softLimit
//                 .forwardSoftLimit(ArmConstants.FORWARD_LIMIT.baseUnitMagnitude())
//                 .reverseSoftLimit(ArmConstants.REVERSE_LIMIT.baseUnitMagnitude())
//                 .forwardSoftLimitEnabled(true)
//                 .reverseSoftLimitEnabled(true)
//                 .apply(armConfig.softLimit);
        
//         armMotor.configure(
//                 armConfig, 
//                 ResetMode.kResetSafeParameters, 
//                 PersistMode.kPersistParameters);
//     }

// }
