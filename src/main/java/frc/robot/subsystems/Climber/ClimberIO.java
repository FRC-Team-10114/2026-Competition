package frc.robot.subsystems.Climber;

import edu.wpi.first.units.measure.Angle;

public interface ClimberIO {
    
    public void setPosition(Angle round);

    public Angle getPosition();

    public void resetPosition();

    public void configure();
}
