package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.IDs;
import frc.robot.subsystems.Intake.IntakeConstants.ArmConstants;

public class ClimberIOTalon implements ClimberIO {

    private final TalonFX climberMotor;
    private final StatusSignal<Angle> climberPosition;
    private final PositionVoltage output;

    public ClimberIOTalon() {
        this.climberMotor = new TalonFX(IDs.Climber.CLIMBER_MOTOR, "canivore");
        this.climberPosition = climberMotor.getPosition();
        this.output = new PositionVoltage(0.0);

        configure();
    }

    @Override
    public void setPosition(Distance meter) {
        double position = meter.baseUnitMagnitude() / ClimberConstants.ROTATION_PER_METER;
        this.climberMotor.setControl(output.withPosition(position));
    }

    @Override
    public Distance getPosition() {
        this.climberPosition.refresh();
        Distance position = Meters.of(climberPosition.getValueAsDouble() * ClimberConstants.ROTATION_PER_METER);

        return position;
    }

    @Override
    public void resetPosition() {
        this.climberMotor.getConfigurator().setPosition(0.0);
    }

    @Override
    public void configure() {
        var climberConfig = new TalonFXConfiguration();

        climberConfig.CurrentLimits
                .withStatorCurrentLimit(null)
                .withStatorCurrentLimitEnable(false)
                .withSupplyCurrentLimit(null)
                .withSupplyCurrentLimitEnable(false);
        climberConfig.MotorOutput
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        climberConfig.Feedback
                .withSensorToMechanismRatio(ClimberConstants.GEAR_RATIO);
        climberConfig.Slot0
                .withKP(0)
                .withKI(0)
                .withKD(0)
                .withGravityType(GravityTypeValue.Elevator_Static);
        climberConfig.HardwareLimitSwitch
                .withForwardLimitAutosetPositionEnable(true)
                .withForwardLimitAutosetPositionValue(ArmConstants.FORWARD_LIMIT)
                .withReverseLimitAutosetPositionEnable(true)
                .withReverseLimitAutosetPositionValue(ArmConstants.REVERSE_LIMIT);
                
    }
    
}
