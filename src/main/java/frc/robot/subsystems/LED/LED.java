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
    
    // 🟢 宣告漸變與爆閃動畫
    private final SingleFadeAnimation fadeAnimation;
    private final StrobeAnimation strobeAnimation;

    private final List<LEDRainbow> Rainbow = new ArrayList<>();

    public LED() {
        this.ledController = new CANdle(IDs.LED.CANDLE, "canivore");

        // 假設你的 LED 燈條長度是 50 顆
        this.rainbowAnimation = new RainbowAnimation(0, 50);
        this.fireAnimation = new FireAnimation(0, 50);
        this.colorFlowAnimation = new ColorFlowAnimation(0, 50);
        this.twinkleAnimation = new TwinkleAnimation(0, 50);
        
        // 🟢 初始化漸變與爆閃動畫
        this.fadeAnimation = new SingleFadeAnimation(0, 50);
        this.strobeAnimation = new StrobeAnimation(0, 50);

        setRainbow();
    }

    public void setRainbow() {
        // Speed 範圍是 0.0 ~ 1.0 (預設通常是 1.0)
        ledController.setControl(rainbowAnimation.withBrightness(0.05));
    }

    public void setFire() {
        // 🔴 小提醒：你原本設定 withBrightness(0.0)，這會讓火焰完全看不見(亮度為0)！
        // 建議改成 0.5 甚至 1.0 才會有燃燒的效果喔！
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

    // ==========================================
    // 🟢 新增：漸變效果 (呼吸燈)
    // ==========================================
    public void setFade(RGBWColor color) {
        // withSpeed 控制呼吸的快慢，數字越小呼吸越慢 (例如 0.2 很適合待機狀態)
        ledController.setControl(fadeAnimation.withColor(color));
    }

    // ==========================================
    // 🟢 新增：爆閃效果 (Strobe)
    // ==========================================
    public void setStrobe(RGBWColor color) {
        // withSpeed 控制閃爍頻率，數字越大閃得越瞎眼 (例如 0.8~1.0 適合警告提示)
        ledController.setControl(strobeAnimation.withColor(color).withFrameRate(50));
    }
    public void setLoadingFlow(RGBWColor color) {
        // withSpeed 控制流水加載的速度 (0.0 ~ 1.0)
        // 數字越小，加載感越慢、越沉穩 (例如 0.4 非常適合攀爬)
        ledController.setControl(colorFlowAnimation.withColor(color).withFrameRate(500));
    }
}