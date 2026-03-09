// package frc.robot.subsystems.Climber;

// import static edu.wpi.first.units.Units.Meters;

// import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.GravityTypeValue;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.units.measure.Angle;
// import frc.robot.Constants.IDs;
// import frc.robot.subsystems.Intake.IntakeConstants.ArmConstants;

// public class ClimberIOTalon implements ClimberIO {

//     private final TalonFX climberMotor;
//     private final StatusSignal<Angle> climberPosition;
//     private final PositionVoltage output;

//     public ClimberIOTalon() {
//         this.climberMotor = new TalonFX(IDs.Climber.CLIMBER_MOTOR, "canivore");
//         this.climberPosition = climberMotor.getPosition();
//         this.output = new PositionVoltage(0.0);

//         configure();
//     }

//     @Override
//     public void setPosition(Angle round) {
//         this.climberMotor.setControl(output.withPosition(round));
//     }

//     @Override
//     public Angle getPosition() {
//         this.climberPosition.refresh();
//         return climberPosition.getValue();
//     }

//     @Override
//     public void resetPosition() {
//         this.climberMotor.getConfigurator().setPosition(0.0);
//     }

//     @Override
//     public void configure() {
//         var climberConfig = new TalonFXConfiguration();

//         climberConfig.CurrentLimits
//                 .withStatorCurrentLimit(ClimberConstants.STATOR_CURRENT_LIMIT)
//                 .withStatorCurrentLimitEnable(true)
//                 .withSupplyCurrentLimit(ClimberConstants.SUPPLY_CURRENT_LIMIT)
//                 .withSupplyCurrentLimitEnable(true);
//         climberConfig.MotorOutput
//                 .withInverted(InvertedValue.Clockwise_Positive)
//                 .withNeutralMode(NeutralModeValue.Brake);
//         climberConfig.Feedback
//                 .withSensorToMechanismRatio(ClimberConstants.GEAR_RATIO);
//         climberConfig.Slot0
//                 .withKP(0.15)
//                 .withKI(0)
//                 .withKD(0)
//                 .withGravityType(GravityTypeValue.Elevator_Static);
//         climberConfig.HardwareLimitSwitch
//                 .withForwardLimitAutosetPositionEnable(true)
//                 .withForwardLimitAutosetPositionValue(ArmConstants.FORWARD_LIMIT)
//                 .withReverseLimitAutosetPositionEnable(true)
//                 .withReverseLimitAutosetPositionValue(ArmConstants.REVERSE_LIMIT);
                
//     }
    
// }
