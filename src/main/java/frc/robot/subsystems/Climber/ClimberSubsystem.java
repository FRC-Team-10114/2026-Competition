package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Rotations;


public class ClimberSubsystem {
    
    private final ClimberIO climber;

    public ClimberSubsystem(ClimberIO climber) {
        this.climber = climber;
    }

    public static ClimberSubsystem create() {
        return new ClimberSubsystem(new ClimberIOSpark());
    }

    public void up() {
        this.climber.setPosition(Rotations.of(10));
    }

    public void down() {
        this.climber.setPosition(Rotations.of(0));
    }
}
