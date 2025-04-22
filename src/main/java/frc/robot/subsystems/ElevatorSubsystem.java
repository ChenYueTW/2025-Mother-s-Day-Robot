package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.LevelPosition.ElevatorLevel;
import frc.robot.lib.motor.TalonModule;
import frc.robot.lib.subsystems.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonModule left = new TalonModule(15, false, true);
    private final TalonModule right = new TalonModule(16, false, true);
    private final Follower follower = new Follower(15, true);

    // SysId data test
    private final VoltageOut voltagRequire = new VoltageOut(0.0);
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(2).per(Second), Volts.of(5),
            null, (state) -> SignalLogger.writeString("state", state.toString())),
        new SysIdRoutine.Mechanism(
            (volts) -> {
                this.left.setControl(voltagRequire.withOutput(volts.in(Volts)));
                this.right.setControl(voltagRequire.withOutput(-volts.in(Volts)));
            },
            null,
            this
        )
    );

    // Gear ratio
    private final double VELOCITY_FACTORY = 2.0 * Units.inchesToMeters(0.938) * Math.PI / 8.0;
    private final double POSITION_FACTOR = 1.0 / 8.0;

    // PID Control and Feed Forward
    public final PIDController lifterPid = new PIDController(6.5, 0.0, 0.0);
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(
            0.022483, 0.38914, 0.12187, 0.0030294);

    // Constants
    private final double MIN_ROTATIONS = 0.0;
    private final double MAX_ROTATIONS = 4.9445068359375;
    private final double TOLERANCE = 0.08;
    private double feedbackVoltage = 0.0;
    private double feedforwardVoltage = 0.0;

    /**
     * Constructs a new ElevatorSubsystem instance.
     * Initializes the subsystem, registers alerts for both left and right motors being connected,
     * and sets the tolerance for the elevator PID controller.
     * The PID data is pushed to the Shuffleboard for visualization and debugging.
     */
    public ElevatorSubsystem() {
        super("Elevator");
        this.registerAlert(this.left.isConnected(), this.right.isConnected());
        this.lifterPid.setTolerance(this.TOLERANCE);

        // NetworkTable grabs ShuffleBoard PID value.
        SmartDashboard.putNumber("Elevator/PID/kP", this.lifterPid.getP());
        SmartDashboard.putNumber("Elevator/PID/kI", this.lifterPid.getI());
        SmartDashboard.putNumber("Elevator/PID/kD", this.lifterPid.getD());
        SmartDashboard.putNumber("Elevator/PID/Setpoint", this.lifterPid.getSetpoint());
        SmartDashboard.putNumber("Elevator/PID/Tolerance", TOLERANCE);
    }

    /**
     * Returns the current position of the elevator axis.
     * Units: Rotation
     *
     * @return The current position of the elevator axis.
     */
    public double getPosition() {
        if (Robot.isSimulation()) this.left.getPositionValue();
        return this.left.getPositionValue() * this.POSITION_FACTOR;
    }

    /**
     * Returns the current velocity of the elevator axis.
     * Units: Meters per second
     *
     * @return The current velocity of the elevator axis.
     */
    public double getVelocity() {
        return (this.left.getVelocityValue() + this.right.getVelocityValue()) / 2.0 * this.VELOCITY_FACTORY;
    }

    /**
     * Returns the current average voltage of the elevator motor.
     *
     * @return The current average voltage of the elevator motor.
     */
    public double getVoltage() {
        return (this.left.getMotorVoltage().getValueAsDouble() + this.right.getMotorVoltage().getValueAsDouble()) / 2.0;
    }

    /**
     * Limit the elevaotr angle within the range.
     *
     * @param speed Set elevator speed.
     */
    public void execute(double speed) {
        if (this.getPosition() >= this.MIN_ROTATIONS && this.getPosition() <= this.MAX_ROTATIONS) {
            this.left.set(speed);
            this.right.setControl(this.follower);
        } else if (this.getPosition() > this.MAX_ROTATIONS && speed >= 0.0) {
            this.left.set(speed);
            this.right.setControl(this.follower);
        } else if (this.getPosition() < this.MIN_ROTATIONS && speed <= 0.0) {
            this.left.set(speed);
            this.right.setControl(this.follower);
        } else {
            this.stopModules();
        }
    }

    /**
     * Limit the elevator angle within the range and use clamp to limit voltage.
     *
     * @param voltage Set elevator voltage.
     */
    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -6.0, 10.0);
        if (this.getPosition() > this.MAX_ROTATIONS - this.TOLERANCE && voltage >= 0.0) {
            voltage = 0.0;
        } else if (this.getPosition() < this.MIN_ROTATIONS + this.TOLERANCE && voltage <= 0.0) {
            voltage = 0.0;
        }
        this.left.setVoltage(voltage);
        this.right.setControl(this.follower);
    }

    /**
     * A command to move the elevator to a target position specified by the PID controller setpoint.
     * This command uses a combination of feedforward and feedback (PID control)
     * to calculate the required voltage.
     *
     * @return A command that moves the elevator to the setpoint and stops when interrupted.
     */
    public Command moveTo() {
        return this.runEnd(() -> {
            this.feedforwardVoltage = this.elevatorFeedforward.calculate(this.lifterPid.getSetpoint());
            this.feedbackVoltage = this.lifterPid.calculate(this.getPosition());
            this.setVoltage(this.feedforwardVoltage + this.feedbackVoltage);
        }, this::stopModules)
            .withName("Elevator.moveTo");
    }

    /**
     * Moves the elevator to the specified target angle using a PID controller.
     * The command resets the PID, sets the target angle,
     * and moves until the target is reached or after 3 seconds.
     *
     * @param angle Target angle to move the elevator to.
     * @return A command to perform the movement.
     */
    public Command moveTo(double angle) {
        return Commands.sequence(
            runOnce(() -> this.lifterPid.reset()),
            runOnce(() -> this.lifterPid.setSetpoint(angle)),
            this.moveTo()
                .until(this.lifterPid::atSetpoint)
                .withTimeout(3.0))
                .withName("Elevator.moveTo");
    }

    /**
     * Holds the elevator at its current position by setting the PID controller setpoint
     * to the current position and maintaining it.
     *
     * @return A command to hold the elevator at its current position.
     */
    public Command holdPosition() {
        return runOnce(() -> {
            this.lifterPid.reset();
            this.lifterPid.setSetpoint(this.getPosition());
        })
            .andThen(this.moveTo())
            .withName("Elevator.holdPosition");
    }

    /**
     * Holds the elevator at the specified target angle by setting the PID controller's setpoint
     * and maintaining it. The motor voltage is reset to 0 before starting.
     *
     * @param angle Target angle to hold the elevator at.
     * @return A command to hold the elevator at the specified position.
     */
    public Command holdToPosition(double angle) {
        return runOnce(() -> {
            this.stopMoveToPositionAndReset();
            this.lifterPid.setSetpoint(angle);
        })
            .andThen(this.moveTo())
            .withName("Elevator.holdToPosition");
    }

    /**
     * Holds the elevator at the specified target angle until a given condition is met.
     * The PID controller's setpoint is set to the target angle, and the command ends
     * when the supplied condition evaluates to true.
     *
     * @param angle Target angle to hold the elevator at.
     * @param until A condition that determines when to stop holding the position.
     * @return A command to hold the elevator at the specified position until the condition is met.
     */
    public Command holdToPositionUntil(double angle, Supplier<Boolean> until) {
        return runOnce(() -> {
            this.stopMoveToPositionAndReset();
            this.lifterPid.setSetpoint(angle);
        })
            .andThen(this.moveTo()
                .until(() -> until.get()))
                .withName("Elevator.holdToPositionUntil");
    }

    public Command autoHoldToPosition(ElevatorLevel elevatorLevel) {
        return runOnce(() -> {
            this.stopMoveToPositionAndReset();
            this.lifterPid.setSetpoint(elevatorLevel.get());
        })
            .andThen(this.moveTo())
            .withName("Elevator.auto.holdToHalfPosition");
    }

    public Command autoHoldToPosition(double angle) {
        return runOnce(() -> {
            this.stopMoveToPositionAndReset();
            this.lifterPid.setSetpoint(angle);
        })
            .andThen(this.moveTo())
            .withName("Elevator.auto.holdToHalfPosition");
    }

    public Command autoHoldToHalfPosition(ElevatorLevel elevatorLevel) {
        return runOnce(() -> {
            this.stopMoveToPositionAndReset();
            this.lifterPid.setSetpoint(elevatorLevel.get() / 2.0);
        })
            .andThen(this.moveTo())
            .withName("Elevator.auto.holdToHalfPosition");
    }

    /**
     * Stops the elevator movement and resets the PID controller.
     *
     * @return A command to stop the elevator and reset the PID controller.
     */
    public Command stopMoveToPositionAndReset() {
        return runOnce(() -> {
            this.lifterPid.reset();
            this.setVoltage(0.0);
        })
            .withName("Elevator.stopMoveToPositionAndReset");
    }

    /**
     * Creates a command that waits until the PID controller reaches its setpoint.
     *
     * @return A command that completes when the PID controller is at the setpoint.
     */
    public Command atSetpoint() {
        return new WaitUntilCommand(() -> this.lifterPid.atSetpoint());
    }

    public Command lifterCanMove() {
        return new WaitUntilCommand(() -> (Math.abs(0.55950927734375 - this.getPosition()) < this.TOLERANCE));
    }

    public double getSetpoint() {
        return this.lifterPid.getSetpoint();
    }

    /**
     * Moves the elevator to a target position specified via dashboard inputs.
     * The PID controller gains (kP, kI, kD) and setpoint are dynamically updated
     * from the dashboard before initiating the movement.
     *
     * @return A command to move the elevator to the dashboard-defined setpoint.
     */
    public Command dashboardMoveTo() {
        return Commands.sequence(
            runOnce(() -> {
                this.lifterPid.setP(SmartDashboard.getNumber("Elevator/PID/kP", 0.0));
                this.lifterPid.setI(SmartDashboard.getNumber("Elevator/PID/kI", 0.0));
                this.lifterPid.setD(SmartDashboard.getNumber("Elevator/PID/kD", 0.0));
                this.lifterPid.setSetpoint(SmartDashboard.getNumber("Elevator/PID/Setpoint", 0.0));
            }),
            this.moveTo()
                .until(this.lifterPid::atSetpoint)
                .withTimeout(3.0))
                .withName("Elevator.DashboardMoveTo");
    }

    /**
     * Holds the elevator at a position specified via dashboard inputs.
     * The PID controller gains (kP, kI, kD) and setpoint are dynamically updated
     * from the dashboard before initiating the hold position command.
     *
     * @return A command to hold the elevator at the dashboard-defined setpoint.
     */
    public Command dashboardHoldTo() {
        return Commands.sequence(
            runOnce(() -> {
                this.lifterPid.setP(SmartDashboard.getNumber("Elevator/PID/kP", 0.0));
                this.lifterPid.setI(SmartDashboard.getNumber("Elevator/PID/kI", 0.0));
                this.lifterPid.setD(SmartDashboard.getNumber("Elevator/PID/kD", 0.0));
                this.lifterPid.setSetpoint(SmartDashboard.getNumber("Elevator/PID/Setpoint", 0.0));
            }),
            this.holdToPosition(SmartDashboard.getNumber("Elevator/PID/Setpoint", 0.0))
                .withName("Elevator.DashboardMoveTo"));
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
     * Performs a complete system identification test for the elevator.
     * The test includes quasistatic and dynamic routines in both upward and downward directions,
     * with waits between each stage. The commands stop when the elevator approaches the defined
     * maximum or minimum rotations.
     *
     * @return A command group to execute the elevator system identification test.
     */
    public Command sysIdElevatorTest() {
        return Commands.sequence(
            // Elevator rising
            this.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
                .raceWith(new WaitUntilCommand(() -> this.getPosition() > this.MAX_ROTATIONS - 0.15)),
            new WaitCommand(1.0),
            // Elevator descending
            this.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
                .raceWith(new WaitUntilCommand(() -> this.getPosition() < this.MIN_ROTATIONS + 0.15)),
            new WaitCommand(1.0),
            // Elevator rising
            this.sysIdDynamic(SysIdRoutine.Direction.kForward)
                .raceWith(new WaitUntilCommand(() -> this.getPosition() > this.MAX_ROTATIONS - 0.15)),
            new WaitCommand(1.0),
            // Elevator descending
            this.sysIdDynamic(SysIdRoutine.Direction.kReverse)
                .raceWith(new WaitUntilCommand(() -> this.getPosition() < this.MIN_ROTATIONS + 0.15)));
    }

    /**
     * Stops the motors for both the left and right elevator modules.
     */
    public void stopModules() {
        this.left.stopMotor();
        this.right.stopMotor();
    }

    /**
     * Updates the SmartDashboard with elevator-related telemetry data.
     */
    @Override
    public void putDashboard() {
        SmartDashboard.putNumber("Elevator/velocity", this.getVelocity());
        SmartDashboard.putNumber("Elevator/position", this.getPosition());
        SmartDashboard.putNumber("Elevator/voltage", this.getVoltage());
        SmartDashboard.putNumber("Elevator/FeedforwardVoltage", this.feedforwardVoltage);
        SmartDashboard.putNumber("Elevator/FeedbackVoltage", this.feedbackVoltage);
        SmartDashboard.putNumber("Elevator/setpoint", this.lifterPid.getSetpoint());
        SmartDashboard.putBoolean("Elevator/atSetpoint", this.lifterPid.atSetpoint());
    }
}
