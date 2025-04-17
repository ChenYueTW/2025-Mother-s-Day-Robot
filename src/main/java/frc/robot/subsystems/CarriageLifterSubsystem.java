package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LevelPosition.LifterLevel;
import frc.robot.lib.motor.TalonModule;
import frc.robot.lib.subsystems.SubsystemBase;

public class CarriageLifterSubsystem extends SubsystemBase {
    private final TalonModule lifter = new TalonModule(18, false, true);

    // SysId data test
    private final VoltageOut voltagRequire = new VoltageOut(0.0);
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(4),
        null, (state) -> SignalLogger.writeString("state-elevator", state.toString())),
        new SysIdRoutine.Mechanism(
            (volts) -> {
                this.lifter.setControl(voltagRequire.withOutput(volts.in(Volts)));
            },
            null,
            this
        )
    );

    // PID Control and Feed Forward
    private final PIDController lifterPid = new PIDController(55.0, 0.0, 0.0);
    private final ArmFeedforward armFeedforward = new ArmFeedforward(
        0.19505, 0.010765, 0.10056, 0.0022567);

    // Constants
    private final double MAX_ROTATIONS = 0.517181396484375;
    private final double MIN_ROTATIONS = 0.0;
    private final double GEAR_RATIO = 90.7;
    private final double TOLERANCE = 0.006;
    private double feedbackVoltage = 0.0;
    private double feedforwordVoltage = 0.0;

    /**
     * Initializes the CarriageLifter subsystem, sets up its position, PID controller, 
     * and exposes PID parameters to SmartDashboard for real-time adjustments.
     */
    public CarriageLifterSubsystem() {
        super("CarrageLifter");
        this.registerAlert(this.lifter.isConnected());
        this.lifterPid.setTolerance(this.TOLERANCE);

        // NetworkTable grabs ShuffleBoard PID value.
        SmartDashboard.putNumber("CarriageLifter/PID/kP", this.lifterPid.getP());
        SmartDashboard.putNumber("CarriageLifter/PID/kI", this.lifterPid.getI());
        SmartDashboard.putNumber("CarriageLifter/PID/kD", this.lifterPid.getD());
        SmartDashboard.putNumber("CarriageLifter/PID/Setpoint", this.lifterPid.getSetpoint());
    }

    /**
     * Returns the current position of the carriage lifter axis.
     * Units: Rotation
     * 
     * @return The current position of the carriage lifter axis.
     */
    public double getPosition() {
        return this.lifter.getPositionValue() / this.GEAR_RATIO;
    }

    public double getVelocity() {
        return this.lifter.getVelocityValue();
    }

    /**
     * Limit the carriage lifter angle within the range.
     * 
     * @param speed Set carriage lifter speed.
     */
    public void execute(double speed) {
        if (this.getPosition() >= this.MIN_ROTATIONS && this.getPosition() <= this.MAX_ROTATIONS) {
            this.lifter.set(speed);
        } else if (this.getPosition() >= this.MAX_ROTATIONS && speed <= 0.0) {
            this.lifter.set(speed);
        } else if (this.getPosition() <= this.MIN_ROTATIONS && speed >= 0.0) {
            this.lifter.set(speed);
        } else {
            this.lifter.stopMotor();
        }
    }

    /**
     * Limit the carriage lifter angle within the range and use clamp to limit voltage.
     * 
     * @param voltage Set carriage lifter voltage.
     */
    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -8.0, 8.0);

        if (this.getPosition() > this.MAX_ROTATIONS && voltage >= 0.0) {
            voltage = 0.0;
        } else if (this.getPosition() < this.MIN_ROTATIONS && voltage <= 0.0) {
            voltage = 0.0;
        }
        this.lifter.setVoltage(voltage);
    }

    /**
     * Moves the carriage to the desired position using a combination of feedforward and feedback control.
     * The feedforward voltage is calculated based on the current position and the setpoint, while the feedback 
     * voltage is computed using the PID controller. The resulting voltages are combined and used to control the motor.
     * 
     * @return A command that, when executed, will move the carriage to the target position.
     */
    public Command moveTo() {
        return runEnd(() -> {
            this.feedforwordVoltage = this.armFeedforward.calculate(
                Units.rotationsToRadians(this.getPosition()), Units.rotationsToRadians(this.lifterPid.getSetpoint()));
            this.feedbackVoltage = this.lifterPid.calculate(this.getPosition());
            this.setVoltage(this.feedforwordVoltage + this.feedbackVoltage);
        }, this::stopModules)
            .withName("CarriageLifter.moveTo");
    }

    /**
     * Moves the carriage to the specified angle using PID control.
     * 
     * @param angle The target angle to move the carriage to.
     * @return A command that, when executed, moves the carriage to the specified angle.
     */
    public Command moveTo(double angle) {
        return Commands.sequence(
            runOnce(() -> {
                this.setVoltage(0.0);
                this.lifterPid.reset();
                this.lifterPid.setSetpoint(angle);
            }),
            this.moveTo()
                .until(this.lifterPid::atSetpoint)
                .withTimeout(3.0))
            .withName("CarriageLifter.moveTo");
    }

    public Command holdPosition() {
        return runOnce(() -> {
            this.lifterPid.reset();
            this.lifterPid.setSetpoint(this.getPosition());
        })
            .andThen(this.moveTo())
            .withName("CarriageLifter.holdPosition");
    }

    public Command holdToPosition(double angle) {
        return runOnce(() -> {
            this.stopMoveToPositionAndReset();
            this.lifterPid.setSetpoint(angle);
        })
            .andThen(this.moveTo())
            .withName("CarriageLifter.holdToPosition");
    }

    public Command autoMoveToPosition(LifterLevel lifterLevel) {
        return this.moveTo(lifterLevel.get());
    }

    public Command autoHoldToPosition(LifterLevel lifterLevel) {
        return this.holdToPosition(lifterLevel.get());
    }

    public Command autoMoveToPosition(double angle) {
        return this.moveTo(angle);
    }

    /**
     * Creates a command that waits until the carriage reaches its setpoint.
     * 
     * @return A command that waits for the carriage to reach the setpoint.
     */
    public Command atSetpoint() {
        return new WaitUntilCommand(() -> this.lifterPid.atSetpoint());
    }

    public Command stopMoveToPositionAndReset() {
        return runOnce(() -> {
            this.lifterPid.reset();
            this.setVoltage(0.0);
        })
            .withName("CarriageLifter.stopMoveToPositionAndReset");
    }

    /**
     * Moves the carriage lifter to a target position specified via dashboard inputs.
     * The PID controller gains (kP, kI, kD) and setpoint are dynamically updated
     * from the dashboard before initiating the movement.
     * 
     * @return A command to move the carriage lifter to the dashboard-defined setpoint.
     */
    public Command dashboardMoveTo() {
        return Commands.sequence(
            runOnce(() -> {
                this.lifterPid.setP(SmartDashboard.getNumber("CarriageLifter/PID/kP", 0.0));
                this.lifterPid.setI(SmartDashboard.getNumber("CarriageLifter/PID/kI", 0.0));
                this.lifterPid.setD(SmartDashboard.getNumber("CarriageLifter/PID/kD", 0.0));
                this.lifterPid.setSetpoint(SmartDashboard.getNumber("CarriageLifter/PID/Setpoint", 0.0));
            }),
            this.moveTo()
                .until(this.lifterPid::atSetpoint)
                .withTimeout(3.0))
            .withName("CarriageLifter.DashboardMoveTo");
    }

    /**
     * Executes a quasistatic system identification routine in the specified direction.
     * 
     * @param direction The direction in which the quasistatic routine should be performed.
     * @return A command to execute the quasistatic system identification routine.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return this.sysIdRoutine.quasistatic(direction);
    }
    
    /**
     * Executes a dynamic system identification routine in the specified direction.
     * 
     * @param direction The direction in which the dynamic routine should be performed.
     * @return A command to execute the dynamic system identification routine.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return this.sysIdRoutine.dynamic(direction);
    }

    /**
     * Performs a complete system identification test for the carriage lifter.
     * The test includes quasistatic and dynamic routines in both upward and downward directions,
     * with waits between each stage. The commands stop when the elevator approaches the defined 
     * maximum or minimum rotations.
     * 
     * @return A command group to execute the elevator system identification test.
     */
    public Command sysIdCarrageLifterTest() {
        return Commands.sequence(
            // Carriage lifter forward
            this.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
                .raceWith(new WaitUntilCommand(() -> this.getPosition() > this.MAX_ROTATIONS - 0.01)),
            new WaitCommand(1.0),
            // Carriage lifter backward
            this.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
                .raceWith(new WaitUntilCommand(() -> this.getPosition() < this.MIN_ROTATIONS + 0.01)),
            new WaitCommand(1.0),
            // Carriage lifter forward
            this.sysIdDynamic(SysIdRoutine.Direction.kForward)
                .raceWith(new WaitUntilCommand(() -> this.getPosition() > this.MAX_ROTATIONS - 0.01)),
            new WaitCommand(1.0),
            // Carriage lifter backward
            this.sysIdDynamic(SysIdRoutine.Direction.kReverse)
                .raceWith(new WaitUntilCommand(() -> this.getPosition() < this.MIN_ROTATIONS + 0.01))
        );
    }

    /**
     * Stops the lifter motor to halt any movement.
     */
    public void stopModules() {
        this.lifter.stopMotor();
    }

    /**
     * Pushes the current status of the CarriageLifter subsystem to the SmartDashboard.
     */
    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("CarriageLifter/position", this.getPosition());
        SmartDashboard.putNumber("CarriageLifter/voltage", this.lifter.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("CarriageLifter/FeedforwardVoltage", this.feedforwordVoltage);
        SmartDashboard.putNumber("CarriageLifter/FeedforbackVoltage", this.feedbackVoltage);
        SmartDashboard.putNumber("CarriageLifter/setpoint", this.lifterPid.getSetpoint());
        SmartDashboard.putBoolean("CarriageLifter/atSetpoint", this.lifterPid.atSetpoint());
    }
}
