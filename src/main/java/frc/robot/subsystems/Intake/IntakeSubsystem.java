package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.Arm.ArmIO;
import frc.robot.subsystems.Intake.Arm.ArmIOTalon;
import frc.robot.subsystems.Intake.Roller.RollerIO;
import frc.robot.subsystems.Intake.Roller.RollerIOSpark;
import frc.robot.subsystems.Intake.Roller.RollerIOTalon;

public class IntakeSubsystem extends SubsystemBase {

    private final ArmIO arm;

    private final RollerIO roller;

    private intakestate state = intakestate.none;

    public IntakeSubsystem(ArmIO arm, RollerIO roller) {
        this.arm = arm;
        this.roller = roller;
    }

    public static IntakeSubsystem create() {
        return new IntakeSubsystem(
                new ArmIOTalon(),
                new RollerIOSpark());
    }

    public enum intakestate {
        intake, none;
    }

    @Override
    public void periodic() {
        // Logger.recordOutput("intakearmangle", this.arm.getPosition());

    }

    public boolean isCanClimb() {
        if (this.arm.getPosition() >= 130) {
            return true;
        } else {
            return false;
        }
    }

    public void rollerStart() {
        this.roller.setVoltage(Volts.of(5.5));
    }

    public void rollerEnd() {
        this.roller.setVoltage(Volts.of(0));
    }

    public void armUp() {
        state = intakestate.none;
        this.arm.setPosition(Degrees.of(82));
    }

    public void armDownForShoot() {
        this.arm.setPosition(Degrees.of(82));
    }

    public void armUpForShoot() {
        this.arm.setPosition(Degrees.of(72));
    }

    public void armDown() {
        state = intakestate.intake;
        this.arm.setPosition(Degrees.of(-2.5));
    }

    public void armUpForClimb() {
        this.arm.setPosition(Degrees.of(135));
    }

    public Command intake() {
        return Commands.sequence(
                Commands.runOnce(this::armDown, this),
                Commands.runOnce(this::rollerStart, this));
    }

    public Command swingIntake() {
        return Commands.either(
            Commands.repeatingSequence(
                Commands.runOnce(this::armDownForShoot, this).withTimeout(0.2),
                Commands.runOnce(this::armUpForShoot, this).withTimeout(0.2)),
            Commands.none(), 
            () -> state == intakestate.none);

    }

    public Command takeIntakeBack() {
        return Commands.sequence(
                Commands.runOnce(this::rollerEnd, this),
                Commands.runOnce(this::armUpForClimb, this)
                );
    }

    public Command sysid() {
        return this.arm.sysid();
    }
}
