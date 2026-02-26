package frc.robot.subsystems.Intake.Arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.IDs;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeConstants.ArmConstants;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class ArmIOTalon implements ArmIO {
    
    private final TalonFX armMotor;

    private final StatusSignal<Angle> armPosition;

    private final MotionMagicVoltage armOutput;

    public ArmIOTalon() {

        this.armMotor = new TalonFX(IDs.Intake.ARM_MOTOR, "canivore");
        this.armPosition = armMotor.getPosition();

        this.armOutput = new MotionMagicVoltage(0.0);

        configure();
        resetEncoder();
    }

    @Override
    public void setPosition(double position) {
        this.armMotor.setControl(this.armOutput.withPosition(position));
    }

    @Override
    public double getPosition() {
        this.armPosition.refresh();
        return this.armPosition.getValueAsDouble();
    }

    @Override
    public void resetEncoder() {
        this.armMotor.getConfigurator().setPosition(0);
    }

    @Override
    public void configure() {
        var armConfig = new TalonFXConfiguration();

        // 1. 基礎馬達與感測器設定
        armConfig.Feedback
                .withSensorToMechanismRatio(ArmConstants.GEAR_RATIO);
        armConfig.MotorOutput
                .withInverted(InvertedValue.Clockwise_Positive);

        // 2. 電流限制 (你原本寫得很好，保留)
        armConfig.CurrentLimits
                .withStatorCurrentLimit(ArmConstants.STATOR_CURRENT_LIMIT.in(Amps)) 
                .withSupplyCurrentLimit(ArmConstants.SUPPLY_CURRENT_LIMIT.in(Amps))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true);

        // 3. 軟體極限設定 (新增！極度建議加入這兩個 Constant)
        // 假設 0 是最低點，0.25 (四分之一圈/90度) 是最高點
        armConfig.SoftwareLimitSwitch
                .withForwardSoftLimitThreshold(ArmConstants.FORWARD_LIMIT) 
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(ArmConstants.REVERSE_LIMIT)
                .withReverseSoftLimitEnable(true);

        // 4. PID 控制與重力前饋
        armConfig.Slot0
                .withKP(ArmConstants.PID[0])
                .withKI(ArmConstants.PID[1])
                .withKD(ArmConstants.PID[2])
                .withGravityType(GravityTypeValue.Arm_Cosine);

        // 5. Motion Magic (強烈建議取消註解並使用)
        armConfig.MotionMagic
                .withMotionMagicAcceleration(ArmConstants.MAX_ACCELERATION)
                .withMotionMagicCruiseVelocity(ArmConstants.CRUISE_VELOCITY);

        // 關閉硬體極限的 Autoset (維持你原本的設定)
        armConfig.HardwareLimitSwitch
                .withForwardLimitAutosetPositionEnable(false)
                .withReverseLimitAutosetPositionEnable(false);

        // 應用設定
        armMotor.getConfigurator().apply(armConfig);
    }
}
