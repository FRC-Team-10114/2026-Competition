package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IDs;

public class ClimberIOSpark implements ClimberIO {

        private final SparkFlex master;
        private final SparkFlex slave;
        private final RelativeEncoder climberEncoder;
        private final SparkClosedLoopController climberController;
        private final SparkClosedLoopController climberController2;

        public ClimberIOSpark() {
                this.master = new SparkFlex(IDs.Climber.CLIMBER_MOTOR, MotorType.kBrushless);
                this.slave = new SparkFlex(IDs.Climber.CLIMBER_SLAVE_MOTOR, MotorType.kBrushless);

                this.climberEncoder = master.getEncoder();
                this.climberController = master.getClosedLoopController();
                this.climberController2 = slave.getClosedLoopController();

                configure();
                resetPosition();
        }

        @Override
        public void setPosition(double round) {
                this.climberController.setSetpoint(round,
                                ControlType.kPosition);
                this.climberController2.setSetpoint(round,
                                ControlType.kPosition);
        }

        @Override
        public double getPosition() {
                return this.climberEncoder.getPosition();
        }

        @Override
        public void setVolt(double voltage) {
                this.master.setVoltage(voltage);
                this.slave.setVoltage(voltage);
        }

        @Override
        public void resetPosition() {
                this.climberEncoder.setPosition(0.0);
        }

        @Override
        public double maingetOutputCurrent() {
                return this.master.getOutputCurrent();
        }

        @Override
        public double getOutputCurrent() {
                return this.slave.getOutputCurrent();
        }

        @SuppressWarnings("removal")
        @Override
        public void configure() {
                var masterConfig = new SparkFlexConfig();
                var slaveConfig = new SparkFlexConfig();

                masterConfig
                                .idleMode(IdleMode.kBrake)
                                .inverted(true)
                                .smartCurrentLimit((int) ClimberConstants.STATOR_CURRENT_LIMIT.in(Amps));

                masterConfig.encoder
                                .positionConversionFactor(ClimberConstants.POSITION_CONVERSION_FACTOR);

                masterConfig.closedLoop
                                .pid(1.0, 0.0, 0.0)
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

                double cruiseRPM = ClimberConstants.CRUISE_VELOCITY.in(RotationsPerSecond) * 60.0;
                double accelRPM = ClimberConstants.MAX_ACCELERATION.in(RotationsPerSecondPerSecond) * 60.0;

                masterConfig.closedLoop.maxMotion
                                .maxAcceleration(accelRPM)
                                .cruiseVelocity(cruiseRPM)
                                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

                masterConfig.softLimit
                                .forwardSoftLimit(ClimberConstants.FORWARD_LIMIT.in(Rotations))
                                .forwardSoftLimitEnabled(false)
                                .reverseSoftLimit(ClimberConstants.REVERSE_LIMIT.in(Rotations))
                                .reverseSoftLimitEnabled(false);

                slaveConfig
                                .idleMode(IdleMode.kBrake)
                                .inverted(false)
                                .smartCurrentLimit((int) ClimberConstants.STATOR_CURRENT_LIMIT.in(Amps));

                slaveConfig.encoder
                                .positionConversionFactor(ClimberConstants.POSITION_CONVERSION_FACTOR);

                slaveConfig.closedLoop
                                .pid(1.0, 0.0, 0.0)
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

                slaveConfig.closedLoop.maxMotion
                                .maxAcceleration(accelRPM)
                                .cruiseVelocity(cruiseRPM)
                                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

                slaveConfig.softLimit
                                .forwardSoftLimit(ClimberConstants.FORWARD_LIMIT.in(Rotations))
                                .forwardSoftLimitEnabled(false)
                                .reverseSoftLimit(ClimberConstants.REVERSE_LIMIT.in(Rotations))
                                .reverseSoftLimitEnabled(false);

                this.master.configure(masterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                this.slave.configure(slaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
}
