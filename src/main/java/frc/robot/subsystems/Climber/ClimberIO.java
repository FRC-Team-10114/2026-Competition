package frc.robot.subsystems.Climber;

import edu.wpi.first.units.measure.Angle;

public interface ClimberIO {
    
    public void setPosition(double round);

    public double getPosition();

    public void resetPosition();

    public void configure();

    public void setVolt(double voltage);

    public  double maingetOutputCurrent();

    public  double getOutputCurrent();
}
