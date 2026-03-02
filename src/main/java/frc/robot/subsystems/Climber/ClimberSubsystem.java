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

    public climbstate state = climbstate.down;

    public ClimberSubsystem(ClimberIO climber) {
        this.climber = climber;

    }

    public enum climbstate {
        up, down;
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
        return Commands.run(() -> this.climber.setVolt(9), this)
                .until(() -> this.climber.maingetOutputCurrent() >= 42 && this.climber.getOutputCurrent() >= 40)
                .finallyDo(() -> this.climber.setVolt(0));
    }

    public Command up() {
        return Commands.run(() -> this.climber.setVolt(-9), this)
                .withTimeout(1.9)
                .finallyDo(() -> this.climber.setVolt(0));
    }

    public Command climb() {
        return Commands.run(() -> this.climber.setVolt(9), this)
                .withTimeout(1.1)
                .finallyDo(() -> this.climber.setVolt(0));
    }
    public Command climbvolt(){
        return Commands.runEnd(() -> this.climber.setVolt(9), () -> this.climber.setVolt(0), this);
    }

    public Command climbup() {
        return Commands.run(() -> this.climber.setVolt(9), this)
                .withTimeout(1.6)
                .finallyDo(() -> this.climber.setVolt(0));
    }

    public Command climbdown() {
        return Commands.run(() -> this.climber.setVolt(-9), this)
                .withTimeout(0.8)
                .finallyDo(() -> this.climber.setVolt(0));
    }

    public Command climbcollecter() {
        return Commands.defer(() -> {
            if (state == climbstate.down) {
                state = climbstate.up;
                return up();
            } else {
                state = climbstate.down;
                return down();
            }

        }, Set.of(this));
    }

    // public void up(){
    // this.climber.setPosition(85); // -109
    // }
    // public void down() {
    // this.climber.setPosition(0); // -40
    // }
    public void stop() {
        this.climber.setVolt(0);
    }

    public double getPosition() {
        return this.climber.getPosition();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("mainclimb", this.climber.maingetOutputCurrent());
        Logger.recordOutput("mclimb", this.climber.getOutputCurrent());
    }
}
