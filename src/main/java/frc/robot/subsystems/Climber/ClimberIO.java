package frc.robot.subsystems.Climber;

import edu.wpi.first.units.measure.Distance;


public interface ClimberIO {
    
    public void setPosition(Distance meter);

    public Distance getPosition();

    public void resetPosition();

    public void configure();
}
