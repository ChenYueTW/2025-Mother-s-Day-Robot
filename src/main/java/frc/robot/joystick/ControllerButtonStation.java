package frc.robot.joystick;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControllerButtonStation extends Joystick {
    public ControllerButtonStation() {
        super(2);
    }

    public Trigger coralL1() {
        return new Trigger(() -> this.getRawButton(1));
    }

    public Trigger coralL2() {
        return new Trigger(() -> this.getRawButton(2));
    }

    public Trigger coralL3() {
        return new Trigger(() -> this.getRawButton(3));
    }

    public Trigger coralL4() {
        return new Trigger(() -> this.getRawButton(4));
    }

    public Trigger algaeHigh() {
        return new Trigger(() -> this.getRawButton(5));
    }

    public Trigger pushAlgae() {
        return new Trigger(() -> this.getRawButton(6));
    }

    public Trigger algaeLow() {
        return new Trigger(() -> this.getRawButton(7));
    }

    public Trigger funnel() {
        return new Trigger(() -> this.getRawButton(8));
    }

    public Trigger releaseFunnel() {
        return new Trigger(() -> this.getRawButton(9));
    }

    public Trigger pushCoral() {
        return new Trigger(() -> this.getRawButton(10));
    }

    public Trigger stop() {
        return new Trigger(() -> this.getRawButton(11));
    }
}
