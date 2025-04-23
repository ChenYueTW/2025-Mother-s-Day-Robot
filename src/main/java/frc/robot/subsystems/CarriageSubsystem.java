package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.lib.motor.TalonModule;
import frc.robot.lib.sensor.Infrared;
import frc.robot.lib.subsystems.SubsystemBase;

public class CarriageSubsystem extends SubsystemBase {
    private final TalonModule carriage = new TalonModule(17, false, true);
    private final Infrared infrared = new Infrared(0);
    private final double CARRIAGE_SPEED = 0.1;

    /**
     * Constructs a new CarriageSubsystem instance.
     * Initializes the subsystem and registers an alert to check if the carriage is connected.
     */
    public CarriageSubsystem() {
        super("Carriage");
        this.registerAlert(this.carriage.isConnected());
    }

    /**
     * Executes the command to move the carriage at a constant speed.
     * This sets the carriage to the predefined speed (CARRIAGE_SPEED).
     */
    public void execute() {
        this.carriage.set(CARRIAGE_SPEED);
    }

    public void execute(double speed) {
        this.carriage.set(speed);
    }

    /**
     * Releases the carriage by moving it in the opposite direction at a constant speed.
     * This sets the carriage to move with a negative speed (-CARRIAGE_SPEED) for release.
     */
    public void release() {
        this.carriage.set(-CARRIAGE_SPEED);
    }

    public Command releaseCommand() {
        return runEnd(() -> this.release(), this::stopModules);
    }
 
    /**
     * Locks the carriage in place by executing the command to move it at a constant speed.
     * 
     * @return A command to lock the carriage.
     */
    public Command lockCarriage() {
        return runEnd(this::release, this::stopModules);
    }

    /**
     * Unlocks the carriage by moving it in the opposite direction at a constant speed.
     * 
     * @return A command to unlock the carriage.
     */
    public Command unlockCarriage() {
        return runEnd(this::execute, this::stopModules)
            .withTimeout(0.07);
    }

    /**
     * Pushes the coral to the reef by releasing the carriage for a duration of 1 second.
     * 
     * @return A command to push the coral to the reef.
     */
    public Command pushCoralToReef() {
        return runEnd(() -> this.carriage.set(-0.3), this::stopModules)
            .withTimeout(1.0);
    }

    public Command pushCoralToReefWithIR() {
        return runEnd(() -> this.carriage.set(-0.4), this::stopModules)
            .until(() -> !this.getIR())
            .andThen(new WaitCommand(0.3)); // TODO
    }

    public Command getAlgaeFromReef() {
        return runEnd(() -> this.carriage.set(0.3), this::stopModules);
    }

    // TODO
    public Command pushAlgaeToProcessor() {
        return runEnd(() -> this.carriage.set(-0.4), this::stopModules)
            .withTimeout(0.5);
    }

    /**
     * Stores the coral by releasing the carriage and then executing the movement to store it.
     * 
     * @return A command to store the coral.
     */
    public Command storageCoral() {
        return runEnd(this::release, this::stopModules)
            .raceWith(this.getIRCommand())
            .andThen(runEnd(() -> this.execute(), this::stopModules)
                .withTimeout(0.3));
    }

    /**
     * Waits until the infrared sensor detects an object.
     * The command will hold until the infrared sensor returns a true value indicating detection.
     * 
     * @return A command that waits for the infrared sensor to detect an object.
     */
    public Command getIRCommand() {
        return new WaitUntilCommand(() -> this.infrared.get());
    }

    /**
     * Returns the current state of the infrared sensor.
     * This method checks whether the infrared sensor is detecting an object.
     * 
     * @return true if the infrared sensor detects an object, false otherwise.
     */
    public boolean getIR() {
        return this.infrared.get();
    }

    /**
     * Stops the motor of the carriage, halting its movement.
     */
    public void stopModules() {
        this.carriage.stopMotor();
    }

    public void stopRecive() {
        this.carriage.setVoltage(0.0);
    }

    /**
     * Updates the SmartDashboard with the current state of the infrared sensor.
     */
    @Override
    public void putDashboard() {
        SmartDashboard.putBoolean("Carriage/IR", this.getIR());
    }
}
