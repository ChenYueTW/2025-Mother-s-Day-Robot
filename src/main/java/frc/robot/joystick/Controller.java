package frc.robot.joystick;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.SwerveConstants;
import frc.robot.lib.helpers.IDashboardProvider;

public class Controller extends XboxController implements IDashboardProvider {

    public Controller() {
        super(1);
    }

    public double lifterElevator() {
        return -MathUtil.applyDeadband(this.getLeftY(), SwerveConstants.DEAD_BAND) * 0.4;
    }

    public double lifterIntake() {
        return MathUtil.applyDeadband(this.getRightY(), SwerveConstants.DEAD_BAND) * 1;
    }

    public Trigger toggleLockIntakeMode() {
        return new Trigger(this::getAButton);
    }

    public Trigger intake() {
        return new Trigger(this::getBButton);
    }

    public Trigger autoIntake() {
        return new Trigger(this::getYButton);
    }

    @Override
    public void putDashboard() {
    }
}
