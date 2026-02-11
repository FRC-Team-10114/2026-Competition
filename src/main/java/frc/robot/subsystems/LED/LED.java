package frc.robot.subsystems.LED;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDs;
import frc.robot.util.RobotEvent.Event.LEDRainbow;

public class LED extends SubsystemBase {
    
    private final CANdle ledController;

    private final RainbowAnimation rainbowAnimation;
    private final FireAnimation fireAnimation;
    private final ColorFlowAnimation colorFlowAnimation;
    private final TwinkleAnimation twinkleAnimation;

    private final List<LEDRainbow> Rainbow = new ArrayList<>();

    public LED() {
        this.ledController = new CANdle(IDs.LED.CANDLE, "canivore");

        this.rainbowAnimation = new RainbowAnimation(0, 50);
        this.fireAnimation = new FireAnimation(0, 50);
        this.colorFlowAnimation = new ColorFlowAnimation(0, 50);
        this.twinkleAnimation = new TwinkleAnimation(0, 50);

        setRainbow();
    }

    public void setRainbow() {
        ledController.setControl(rainbowAnimation.withBrightness(1));
    }

    public void setFire() {
        ledController.setControl(fireAnimation.withBrightness(1));
    }

    public void close() {
        ledController.close();
    }

    public void setColorFlow() {
        ledController.setControl(colorFlowAnimation.withColor(RGBWColor.fromHSV(24, 100, 100)));
    }

    public void setBlink(RGBWColor color) {
        ledController.setControl(twinkleAnimation.withColor(color));
    }
}
