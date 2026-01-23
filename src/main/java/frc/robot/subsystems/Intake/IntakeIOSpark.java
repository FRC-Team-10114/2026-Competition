package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.IntakeConstants;

public class IntakeIOSpark implements IntakeIO {

    private final SparkMax intake;


    public IntakeIOSpark() {

        this.intake = new SparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

    }

    @Override
    public void setControl(double velocity, boolean isFOC) {
        intake.set(velocity);
    }

    @Override
    public double getVelocity() {
        return this.intake.getEncoder().getVelocity();
    }

    @Override
    public void configure() {

        SparkMaxConfig MaxIntakeConfig = new SparkMaxConfig();

        MaxIntakeConfig
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit(30)
                .apply(MaxIntakeConfig);

        this.intake.configure(
                MaxIntakeConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }
}
