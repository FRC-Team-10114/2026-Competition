package frc.robot.subsystems.LED;

import com.ctre.phoenix6.hardware.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    
    private final CANdle ledController;

    public LED() {
        this.ledController = new CANdle(66);
    }
}
