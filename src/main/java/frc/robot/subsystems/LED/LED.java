package frc.robot.subsystems.LED;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
// 🟢 新增這兩個動畫類別
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;

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
    
    private final SingleFadeAnimation fadeAnimation;
    private final StrobeAnimation strobeAnimation;

    private final List<LEDRainbow> Rainbow = new ArrayList<>();

    public LED() {
        this.ledController = new CANdle(IDs.LED.CANDLE, "canivore");

        this.rainbowAnimation = new RainbowAnimation(0, 50);
        this.fireAnimation = new FireAnimation(0, 50);
        this.colorFlowAnimation = new ColorFlowAnimation(0, 50);
        this.twinkleAnimation = new TwinkleAnimation(0, 50);
        
        this.fadeAnimation = new SingleFadeAnimation(0, 50);
        this.strobeAnimation = new StrobeAnimation(0, 50);

        setRainbow();
    }
    public void setRainbow() {
        ledController.setControl(rainbowAnimation.withBrightness(0.05));
    }

    public void setFire() {
        ledController.setControl(fireAnimation.withBrightness(0.05).withFrameRate(500));
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

    public void setFade(RGBWColor color) {
        ledController.setControl(fadeAnimation.withColor(color));
    }

    public void setStrobe(RGBWColor color) {
        ledController.setControl(strobeAnimation.withColor(color).withFrameRate(50));
    }
    public void setLoadingFlow(RGBWColor color) {
        ledController.setControl(colorFlowAnimation.withColor(color).withFrameRate(500));
    }
}