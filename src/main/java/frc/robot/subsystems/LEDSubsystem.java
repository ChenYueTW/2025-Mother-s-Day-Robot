package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import frc.robot.lib.subsystems.LEDModule;

public class LEDSubsystem extends SubsystemBase {
    private final int LED_PORT = 0;
    private final int LED_BUFFER_LENGTH = 150;

    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;
    private final AddressableLEDBufferView left;
    private final AddressableLEDBufferView right;
    private final LEDModule[] ledModules;
    private boolean idleMode = true;

    public LEDSubsystem() {
        this.led = new AddressableLED(this.LED_PORT);
        this.ledBuffer = new AddressableLEDBuffer(this.LED_BUFFER_LENGTH);
        this.left = this.ledBuffer.createView(0, (this.LED_BUFFER_LENGTH / 2 - 1)).reversed();
        this.right = this.ledBuffer.createView(this.LED_BUFFER_LENGTH / 2, this.LED_BUFFER_LENGTH - 1);

        this.led.setLength(this.LED_BUFFER_LENGTH);
        this.led.setData(this.ledBuffer);
        this.led.start();

        this.ledModules = new LEDModule[] {
                new LEDModule(0, this.LED_BUFFER_LENGTH, false, this.ledBuffer)
        };
    }

    public void setIdleMode(boolean mode) {
        this.idleMode = mode;
    }

    public void rainbow(int step) {
        for (LEDModule ledMoudle : this.ledModules) {
            ledMoudle.rainbow(step);
        }
    }

    public void defaultMode() {
        Distance ledSpacing = Meters.of(1.0 / 180.0);
        LEDPattern base = LEDPattern.gradient(GradientType.kContinuous, new Color(255, 17, 0), Color.kBlack);
        LEDPattern pattern = base.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), ledSpacing);
        pattern.applyTo(this.left);
        pattern.applyTo(this.right);
    }

    public void hasCoral() {
        for (LEDModule ledModule : this.ledModules) {
            ledModule.setAllColorWithHSV(50, 255, 128);
        }
    }

    public void pushingCoral() {
        LEDPattern base = LEDPattern.gradient(GradientType.kContinuous, Color.kRed, Color.kBlue);
        LEDPattern pattern = base.blink(Seconds.of(0.1));
        pattern.applyTo(this.ledBuffer);
    }

    public void funnel() {
        Distance ledSpacing = Meters.of(1.0 / (double) this.LED_BUFFER_LENGTH);
        LEDPattern base = LEDPattern.gradient(GradientType.kContinuous, new Color(255, 72, 0), Color.kBlack);
        LEDPattern pattern = base.scrollAtAbsoluteSpeed(MetersPerSecond.of(1.8), ledSpacing);
        pattern.applyTo(this.left);
        pattern.applyTo(this.right);
    }

    public void autonomous() {
        double percentage = (15.0 - DriverStation.getMatchTime()) / 15.0;
        LEDPattern base = LEDPattern.solid(new Color("#440080"));
        LEDPattern pattern = LEDPattern.progressMaskLayer(() -> percentage).reversed();
        LEDPattern display = base.mask(pattern);
        display.applyTo(this.left);
        display.applyTo(this.right);
    }

    public void setData() {
        this.led.setData(this.ledBuffer);
    }

    @Override
    public void periodic() {
        if (this.idleMode) this.defaultMode();
        this.led.setData(this.ledBuffer);
    }
}
