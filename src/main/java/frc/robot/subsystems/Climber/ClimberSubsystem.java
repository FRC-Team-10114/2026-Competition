package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Meters;


public class ClimberSubsystem {
    
    private final ClimberIO climber;

    public ClimberSubsystem(ClimberIO climber) {
        this.climber = climber;
    }

    public void up() {
        this.climber.setPosition(Meters.of(5));
    }

    public void down() {
        this.climber.setPosition(Meters.of(0.0));
    }
}
