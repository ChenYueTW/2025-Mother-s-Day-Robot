package frc.robot.lib.sensor;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C.Port;

public class ColorSensor extends ColorSensorV3 {
    public ColorSensor(Port port) {
        super(port);
        this.configureColorSensor(
            ColorSensorResolution.kColorSensorRes16bit,
            ColorSensorMeasurementRate.kColorRate25ms,
            GainFactor.kGain1x);
        this.configureProximitySensorLED(
            LEDPulseFrequency.kFreq100kHz,
            LEDCurrent.kPulse125mA, 255);
    }

    public boolean isGetBlue() {
        double red = this.getColor().red;
        double green = this.getColor().green;
        double blue = this.getColor().blue;

        if (red >= 0.15 && red <= 0.25 && green >= 0.38 && green <= 0.5 && blue >= 0.3 && blue <= 0.45) {
            return true;
        }
        return false;
    }

    public boolean isGetRed() {
        double red = this.getColor().red;
        double green = this.getColor().green;
        double blue = this.getColor().blue;

        if (red >= 0.49 && red <= 0.57 && green >= 0.3 && green <= 0.35 && blue >= 0.1 && blue <= 0.16) {
            return true;
        }
        return false;
    }

    public boolean isAlliance() {
        return DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? this.isGetRed() : this.isGetBlue();
    }

    public String getRGB() {
        return this.getColor().red + ", " + this.getColor().green + ", " + this.getColor().blue;
    }
}
