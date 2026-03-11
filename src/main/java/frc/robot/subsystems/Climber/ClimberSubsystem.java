package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Rotations;

import java.nio.file.attribute.PosixFileAttributeView;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Hopper.HopperSubsystem;
import frc.robot.subsystems.Hopper.Spindexer.SpindexerIOHardware;

public class ClimberSubsystem extends SubsystemBase {

    private final ClimberIO climber;

    public climbstate state = climbstate.DOWN;

    public ClimberSubsystem(ClimberIO climber) {
        this.climber = climber;

    }

    public enum climbstate {
        UP, DOWN;
    }

    public static ClimberSubsystem create() {
        return new ClimberSubsystem(new ClimberIOSpark());
    }

    // public void up() {
    // this.climber.setVolt(9);
    // }
    // public void down() {
    // this.climber.setVolt(-9);
    // }

    public Command down() {
        return Commands.run(() -> this.climber.setVoltage(9), this)
                .until(() -> this.climber.getMasterCurrent() >= 42 && this.climber.getSlaveCurrent() >= 40)
                .finallyDo(() -> this.climber.setVoltage(0));
    }

    public Command up() {
        return Commands.run(() -> this.climber.setVoltage(-9), this)
                .withTimeout(1.35)
                .finallyDo(() -> this.climber.setVoltage(0));
    }

    public Command climb() {
        return Commands.run(() -> this.climber.setVoltage(9), this)
                .withTimeout(1.0)
                .finallyDo(() -> this.climber.setVoltage(0));
    }

    public Command manualClimberUp() {
        return Commands.runEnd(() -> this.climber.setVoltage(9), () -> this.climber.setVoltage(0), this);
    }

    public Command manualClimberDown() {
        return Commands.runEnd(() -> this.climber.setVoltage(-9), () -> this.climber.setVoltage(0), this);
    }

    public Command autoClimberUp() {
        return Commands.run(() -> this.climber.setVoltage(9), this)
                .withTimeout(1.6)
                .finallyDo(() -> this.climber.setVoltage(0));
    }

    public Command autoClimberDown() {
        return Commands.run(() -> this.climber.setVoltage(-9), this)
                .withTimeout(0.8)
                .finallyDo(() -> this.climber.setVoltage(0));
    }

    public Command climberCollecter() {
        return Commands.defer(() -> {
            if (state == climbstate.DOWN) {
                state = climbstate.UP;
                return up();
            } else {
                state = climbstate.DOWN;
                return down();
            }

        }, Set.of(this));
    }
    public void stop() {
        this.climber.setVoltage(0);
    }

    public double getPosition() {
        return this.climber.getPosition();
    }

    @Override
    public void periodic() {}
}
