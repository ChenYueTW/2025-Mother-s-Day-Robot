package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.helpers.DashboardHelper;
import frc.robot.lib.helpers.Elastic;

public class Robot extends TimedRobot {
	private Command autonomousCommand;
	private final RobotContainer robotContainer;

	public Robot() {
		super(0.01);

		DashboardHelper.enableRegistration();
		this.robotContainer = new RobotContainer();
		DashboardHelper.disableRegistration();

		CameraServer.startAutomaticCapture("Camera", 0);

		SmartDashboard.putData("Commands Scheduler", CommandScheduler.getInstance());
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		DashboardHelper.putAllRegistries();
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
		RobotContainer.funneling = false;
		RobotContainer.pushingCoral = false;
	}

	@Override
	public void autonomousInit() {
		this.autonomousCommand = this.robotContainer.getAutonomousCommand();
		Elastic.selectTab("Autonomous");
		this.robotContainer.elevatorSubsystem.lifterPid.setP(3.0);

		if (this.autonomousCommand != null) {
			this.autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		Elastic.selectTab(DriverStation.isFMSAttached() ? "Practice" : "Teleoperated");
		this.robotContainer.elevatorSubsystem.lifterPid.setP(6.5);
		if (this.autonomousCommand != null) {
			this.autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void simulationInit() {
	}

	@Override
	public void simulationPeriodic() {
	}
}
