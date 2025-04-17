package frc.robot.joystick;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.SwerveConstants;

public class Driver extends XboxController {
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(SwerveConstants.MAX_ACCELERATION);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(SwerveConstants.MAX_ACCELERATION);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(SwerveConstants.MAX_ANGULAR_ACCELERATION);

    public Driver() {
        super(0);
    }

    public double getXDesiredSpeed() {
        double speed = -MathUtil.applyDeadband(this.getLeftY(), SwerveConstants.DEAD_BAND) * 1.0 * this.getBrake();
        return this.xLimiter.calculate(speed);
    }

    public double getYDesiredSpeed() {
        double speed = -MathUtil.applyDeadband(this.getLeftX(), SwerveConstants.DEAD_BAND) * 1.0 * this.getBrake();
        return this.yLimiter.calculate(speed);
    }

    public double getRotationSpeed() {
        double speed = -MathUtil.applyDeadband(this.getRightX(), SwerveConstants.DEAD_BAND) * 0.9 * this.getBrake();
        return this.rotationLimiter.calculate(speed);
    }

    public double getBrake() {
        return 1.0 - MathUtil.applyDeadband(this.getRightTriggerAxis(), SwerveConstants.DEAD_BAND) * 0.8;
    }

    public Trigger resetGyro() {
        return new Trigger(this::getRightBumperButton);
    }

    public boolean robotMode() {
        return this.getLeftBumperButton();
    }

    public boolean trackLeftTag() {
        return this.getXButton();
    }

    public boolean trackRightTag() {
        return this.getBButton();
    }

    public boolean turnToLeftSourceAngle() {
        return this.getBackButton();
    }

    public boolean turnToRightSourceAngle() {
        return this.getStartButton();
    }

    public int turnToReef() {
        return this.getPOV();
    }
}
