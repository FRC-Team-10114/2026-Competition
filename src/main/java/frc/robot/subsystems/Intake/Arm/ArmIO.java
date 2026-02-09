package frc.robot.subsystems.Intake.Arm;

import edu.wpi.first.units.measure.Angle;

public interface ArmIO {

    public void setPosition(double position);

    public double getPosition();

    public void resetEncoder();

    public void configure();
}
