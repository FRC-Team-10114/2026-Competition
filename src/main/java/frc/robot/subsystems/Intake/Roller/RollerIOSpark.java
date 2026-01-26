package frc.robot.subsystems.Intake.Roller;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.IntakeConstants;

public class RollerIOSpark implements RollerIO {

    private final SparkMax rollerMotor;
    private final RelativeEncoder rollerEncoder;
    private final SparkClosedLoopController rollerController;

    public RollerIOSpark() {
        this.rollerMotor = new SparkMax(IntakeConstants.INTAKE_ROLLER_MOTOR_ID, MotorType.kBrushless);
        this.rollerEncoder = rollerMotor.getEncoder();
        this.rollerController = rollerMotor.getClosedLoopController();
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        this.rollerController.setSetpoint(velocity.baseUnitMagnitude(), ControlType.kVelocity);
    }

    @Override
    public AngularVelocity getVelocity() {
        return RadiansPerSecond.of(this.rollerEncoder.getPosition());
    }

    @Override
    public void configure() {
        var rollerConfig = new SparkMaxConfig();

        rollerConfig
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit((int)IntakeConstants.INTAKE_ARM_SUPPLY_CURRENT_LIMIT.baseUnitMagnitude())
                .apply(rollerConfig);
        rollerConfig.closedLoop
                .pid(
                        IntakeConstants.INTAKE_ROLLER_PID[0], 
                        IntakeConstants.INTAKE_ROLLER_PID[1], 
                        IntakeConstants.INTAKE_ROLLER_PID[2])
                .apply(rollerConfig.closedLoop);
        rollerConfig.encoder
                .velocityConversionFactor(IntakeConstants.INTAKE_ROLLER_VELOCITY_CONVERSION_FACOTR)
                .positionConversionFactor(IntakeConstants.INTAKE_ARM_POSITION_CONVERSION_FACTOR)
                .apply(rollerConfig.encoder);

        rollerMotor.configure(
                rollerConfig, 
                ResetMode.kResetSafeParameters, 
                PersistMode.kPersistParameters);
    }
    
}
