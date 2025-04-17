package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.SwerveConstants;
import frc.robot.lib.helpers.IDashboardProvider;
import frc.robot.lib.math.MathHelper;

public class SwerveSubsystem extends SubsystemBase implements IDashboardProvider {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final AHRS gyro;
    private final SwerveDriveOdometry odometry;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field;

    // Path
    private final PIDController xController = new PIDController(9.1, 0.0, 0.0);
    private final PIDController yController = new PIDController(9.1, 0.0, 0.0);
    private final PIDController headingController = new PIDController(6.5, 0.0, 0.0);

    // Situate Robot
    private final PIDController drivePid = new PIDController(1.1, 0.0, 0.0);
    private final PIDController steerPid = new PIDController(0.022, 0.0, 0.0);
    private final PIDController autoDrivePid = new PIDController(0.8, 0.0, 0.0);

    // Simulation
    private SwerveModuleState[] lastDesiredState = new SwerveModuleState[] {
        new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
    };
    private Pose2d simOdomotry;
    private double timeFromLastUpdate = 0;
	private double lastSimTime = Timer.getFPGATimestamp();
    private double translateX = 0.0;
    private double translateY = 0.0;
    private double simAngle = 0.0;

    // Publishers for simulating and tracking swerve drive poses and module states
    private final StructPublisher<Pose2d> swerveSim = NetworkTableInstance.getDefault()
        .getStructTopic("AdvantageScope/SwervePose", Pose2d.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> moduleSim = NetworkTableInstance.getDefault()
        .getStructArrayTopic("AdvantageScope/SwerveModule", SwerveModuleState.struct).publish();
    private final StructPublisher<Pose2d> swerveNow = NetworkTableInstance.getDefault()
        .getStructTopic("AdvantageScope/RealSwervePose", Pose2d.struct).publish();

    /**
     * Initializes the SwerveSubsystem with modules, odometry, and pose estimator,
     * and configures dashboard data for swerve module positions and velocities.
     **/
    public SwerveSubsystem() {
        this.registerDashboard();
        this.frontLeft = new SwerveModule(
            2, 1, 9,
            false, true,
            "frontLeft"
        );
        this.frontRight = new SwerveModule(
            4, 3, 10,
            true, true,
            "frontRight"
        );
        this.backLeft = new SwerveModule(
            6, 5, 11,
            false, true,
            "backLeft"
        );
        this.backRight = new SwerveModule(
            8, 7, 12,
            true, true,
            "backRight"
        );
        this.gyro = new AHRS(NavXComType.kUSB1);
        this.odometry = new SwerveDriveOdometry(
            SwerveConstants.swerveDriveKinematics, this.getHeading(), this.getModulePosition()
        );
        this.field = new Field2d();
        this.simOdomotry = this.odometry.getPoseMeters();
        this.poseEstimator = new SwerveDrivePoseEstimator(
            SwerveConstants.swerveDriveKinematics, Rotation2d.fromDegrees(-this.getGyroAngle()), this.getModulePosition(), new Pose2d());
        this.headingController.enableContinuousInput(-Math.PI, Math.PI);
        this.steerPid.enableContinuousInput(-180.0, 180.0);
        this.gyro.reset();

        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }
        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getSpeeds,
            (speeds, feedforwards) -> this.autoDriverSwerve(speeds),
            new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.01, 0.0), 
                    new PIDConstants(5.0, 0.0, 0.0)
            ),
            config,
            () -> false,
            this
        );

        // Put swerve to Elastic board.
        SmartDashboard.putData("ElasticSwerve/", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("FrontLeft Posisiton", () -> frontLeft.getPosition().angle.getRadians(), null);
                builder.addDoubleProperty("FrontLeft Velocity", () -> frontLeft.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("FrontRight Posisiton", () -> frontRight.getPosition().angle.getRadians(), null);
                builder.addDoubleProperty("FrontRight Velocity", () -> frontRight.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("BackLeft Posisiton", () -> backLeft.getPosition().angle.getRadians(), null);
                builder.addDoubleProperty("BackLeft Velocity", () -> backLeft.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("BackRight Posisiton", () -> backRight.getPosition().angle.getRadians(), null);
                builder.addDoubleProperty("BackRight Velocity", () -> backRight.getState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Robot Heading", () -> getRotation().getRadians(), null);
            }
        });
    }

    /**
     * Updates the timer by calculating the time elapsed since the last update.
     */
    public void updateTimer() {
        this.timeFromLastUpdate = Timer.getFPGATimestamp() - this.lastSimTime;
		this.lastSimTime = Timer.getFPGATimestamp();
    }

    public void updateSimOdomotry() {
        // SwerveModuleState[] states = this.getModuleStates();
        // ChassisSpeeds speeds = SwerveConstants.swerveDriveKinematics.toChassisSpeeds(states);
        
    }

    /**
     * Updates the timer, publishes the current swerve pose,
     * and updates odometry and pose estimator.
     */
    @Override
    public void periodic() {
        this.updateTimer();
        this.simOdomotry = new Pose2d(this.translateX, this.translateY, new Rotation2d(this.simAngle));
        this.swerveSim.set(this.simOdomotry);
        this.field.setRobotPose(this.getPose());
        this.swerveNow.set(this.getPose());
        this.odometry.update(this.getHeading(), this.getModulePosition());
        this.poseEstimator.update(this.getRotation(), this.getModulePosition());
    }

    @Override
    public void simulationPeriodic() {
        // this.odometry.update(this.gyro.getRotation2d(), this.getModulePosition());
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot    Angular rate of the robot.
     * @param field  Whether the provided x and y speeds are relative to the
     *               field.
     */
    public void driveSwerve(double xSpeed, double ySpeed, double rotation, boolean field) {
        SwerveModuleState[] state = SwerveConstants.swerveDriveKinematics.toSwerveModuleStates(field ? 
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, this.getHeading()) :
            new ChassisSpeeds(xSpeed, ySpeed, rotation)
        );
        this.setModuleState(state);
    }

    /**
     * Converts chassis speeds to swerve module states and sets the module states.
     * 
     * @param speeds The chassis speeds to convert and apply.
     */
    public void autoDriverSwerve(ChassisSpeeds speeds) {
        SwerveModuleState[] states = SwerveConstants.swerveDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 4.2);
        
        this.translateX += (speeds.vxMetersPerSecond * this.timeFromLastUpdate);
        this.translateY += (speeds.vyMetersPerSecond * this.timeFromLastUpdate);
        this.simAngle += speeds.omegaRadiansPerSecond * this.timeFromLastUpdate;

        this.setModuleState(states);
    }

    /**
     * Moves the robot to a specified position and angle based on a given vector.
     * 
     * @param vector The translation vector representing the desired position change. 
     * @param angle The target angle the robot should face.
     */
    public void situateRobot(Translation2d vector, double angle) {
        double speed = this.drivePid.calculate(0.0, vector.getNorm());
        double rotation = this.steerPid.calculate(this.getGyroAngle(), angle);

        speed = MathHelper.applyMax(speed, 0.2);
        rotation = MathHelper.applyMax(rotation, Math.PI);

        double xSpeed = vector.getNorm() == 0 ? 0.0 : speed * vector.getX() / vector.getNorm();
        double ySpeed = vector.getNorm() == 0 ? 0.0 : speed * vector.getY() / vector.getNorm();

        this.driveSwerve(xSpeed, ySpeed, rotation, false);
    }

    public boolean situateRobot(Pose2d targetPose) {
        Translation2d vector = targetPose.getTranslation().minus(this.getPose().getTranslation());

        double speed = this.drivePid.calculate(0.0, vector.getNorm());
        double rotation = this.steerPid.calculate(MathHelper.capPeriodic(this.getHeading().getDegrees(), 360.0), MathHelper.capPeriodic(targetPose.getRotation().getDegrees(), 360.0));

        speed = MathHelper.applyMax(speed, 0.5);
        rotation = MathHelper.applyMax(rotation, Math.PI);

        double xSpeed = vector.getNorm() == 0 ? 0.0 : speed * vector.getX() / vector.getNorm();
        double ySpeed = vector.getNorm() == 0 ? 0.0 : speed * vector.getY() / vector.getNorm();

        this.driveSwerve(xSpeed, ySpeed, rotation, true);

        return vector.getNorm() < 0.02 && Math.abs(MathHelper.capPeriodic(targetPose.getRotation().minus(this.getHeading()).getDegrees(), 360.0)) < 2.0;
    }

    public boolean situateAutoRobot(Pose2d targetPose) {
        Translation2d vector = targetPose.getTranslation().minus(this.getPose().getTranslation());

        double speed = this.autoDrivePid.calculate(0.0, vector.getNorm());
        double rotation = this.steerPid.calculate(MathHelper.capPeriodic(this.getHeading().getDegrees(), 360.0), MathHelper.capPeriodic(targetPose.getRotation().getDegrees(), 360.0));

        speed = MathHelper.applyMax(speed, 2.0);
        rotation = MathHelper.applyMax(rotation, Math.PI / 4.0);

        double xSpeed = vector.getNorm() == 0 ? 0.0 : speed * vector.getX() / vector.getNorm();
        double ySpeed = vector.getNorm() == 0 ? 0.0 : speed * vector.getY() / vector.getNorm();
        double speedNorm = Math.sqrt(xSpeed * xSpeed + ySpeed * ySpeed);
        SmartDashboard.putNumber("SPEED", speed);

        this.driveSwerve(xSpeed, ySpeed, rotation, true);

        return this.getModuleStates()[0].speedMetersPerSecond <= 0.005 && speedNorm <= 0.05;
    }

    /**
     * Method to drive the simulate robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot    Angular rate of the robot.
     */
    public void simSwerve(double xSpeed, double ySpeed, double rotation) {
        SwerveModuleState[] states = SwerveConstants.swerveDriveKinematics.toSwerveModuleStates(
            new ChassisSpeeds(xSpeed, ySpeed, rotation)
        );
        this.setModuleState(states);
        this.moduleSim.set(states);

        ChassisSpeeds speeds = SwerveConstants.swerveDriveKinematics.toChassisSpeeds(states);

        if (speeds.vxMetersPerSecond != 0.0) {
            this.translateX += speeds.vxMetersPerSecond * this.timeFromLastUpdate;
        }
        if (speeds.vyMetersPerSecond != 0.0) {
            this.translateY += speeds.vyMetersPerSecond * this.timeFromLastUpdate;
        }
        if (speeds.omegaRadiansPerSecond != 0.0) {
            this.simAngle += speeds.omegaRadiansPerSecond * this.timeFromLastUpdate;
        }
    }

    /**
     * Returns the current gyro rotation, using simulated values if in simulation mode.
     * 
     * @return The current gyro rotation as a Rotation2d.
     */
    // public Rotation2d getGyroRotation() {
    //     if (Robot.isSimulation() && this.lastDesiredState != null) {
    //         this.simAngle += SwerveConstants.swerveDriveKinematics
    //             .toChassisSpeeds(this.lastDesiredState).omegaRadiansPerSecond * this.timeFromLastUpdate;
            
    //         this.simAngle = this.simAngle % (2 * Math.PI);
    //         // SmartDashboard.putNumber("SimAngle", this.simAngle);

    //         this.simAngle = (this.simAngle < 0) ? this.simAngle + (2 * Math.PI) : this.simAngle;
	// 		return Rotation2d.fromRadians(this.simAngle);
    //     }
    //     return null;
    // }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleState(SwerveModuleState[] states) {
        this.lastDesiredState = states;
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_SPEED);
        this.frontLeft.setDesiredState(states[0]);
        this.frontRight.setDesiredState(states[1]);
        this.backLeft.setDesiredState(states[2]);
        this.backRight.setDesiredState(states[3]);
    }

    /**
     * Returns the current position of the swerve.
     *
     * @return The current position of the swerve.
     */
    public SwerveModulePosition[] getModulePosition() {
        return new SwerveModulePosition[] {
            this.frontLeft.getPosition(),
            this.frontRight.getPosition(),
            this.backLeft.getPosition(),
            this.backRight.getPosition()
        };
    }

    /**
     * Returns the current state of the swerve.
     *
     * @return The current state of the swerve.
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            this.frontLeft.getState(),
            this.frontRight.getState(),
            this.backLeft.getState(),
            this.backRight.getState()
        };
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        if (Robot.isReal()) {
            Pose2d pose = this.odometry.getPoseMeters();
            return pose;
            // return new Pose2d(pose.getX(), pose.getY() - 0.15, pose.getRotation());
        } else {
            return this.simOdomotry;
        }
    }

    public Pose2d getRobotToFieldPose(Translation2d robotToTagPose, Translation2d tagToFieldPose) {
        robotToTagPose = robotToTagPose.rotateBy(this.getHeading());
        Translation2d translation = tagToFieldPose.minus(robotToTagPose);
        return new Pose2d(translation, this.getHeading());
    }

    public void resetPoseFromAprilTag(Pose2d pose) {
        this.odometry.resetPose(pose);
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetPose(Pose2d pose) {
        if (Robot.isReal()) {
            // boolean isBlue = DriverStation.getAlliance().isPresent()
            //     && DriverStation.getAlliance().get() == Alliance.Blue;
            // double angleAdj = isBlue ? -pose.getRotation().getDegrees() : 180.0 - pose.getRotation().getDegrees();
            // this.gyro.setAngleAdjustment(Math.IEEEremainder(angleAdj, 360.0));
            this.gyro.reset();
            this.odometry.resetPosition(this.getHeading(), this.getModulePosition(), pose);
        } else {
            this.simOdomotry = pose;
            this.translateX = pose.getX();
            this.translateY = pose.getY();
            this.simAngle = pose.getRotation().getRadians();
        }
    }

    public boolean isGyroConnected() {
        return this.gyro.isConnected();
    }

    public double getGyroAngle() {
        return -this.gyro.getAngle();
    }

    public Rotation2d getHeading() {
        return new Rotation2d(Units.degreesToRadians(this.getGyroAngle()));
    }

    public Rotation2d getRotation() {
        double angle = this.getGyroAngle();

        if (DriverStation.getAlliance().get() == Alliance.Red) {
            angle += 180.0;
        }

        return new Rotation2d(Units.degreesToRadians(angle));
    }

    /**
     * Returns the chassis speed of the swerve
     * 
     * @return The chassis speed of the swerve
     */
    public ChassisSpeeds getSpeeds() {
        return SwerveConstants.swerveDriveKinematics.toChassisSpeeds(this.getModuleStates());
    }

    /**
     * Follows a trajectory sample by calculating and applying the necessary chassis speeds
     * based on the current pose and the sample's desired values.
     * 
     * @param sample The trajectory sample containing the target values (x, y, omega).
     */
    public void followTrajectory(SwerveSample sample) {
        Pose2d pose = this.getPose();

        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        SmartDashboard.putNumber("Path/xSpeed", pose.getX());
        SmartDashboard.putNumber("Path/xSetpoint", sample.x);
        SmartDashboard.putNumber("Path/rotSpeed", pose.getRotation().getRadians());
        SmartDashboard.putNumber("Path/rotSetpoint", sample.heading);

        this.autoDriverSwerve(speeds);
    }

    /**
     * Resets the gyro to its default state.
     */
    public void resetGyro() {
        this.gyro.reset();
    }

    /**
     * Stops all swerve drive modules.
     */
    public void stopModules() {
        this.frontLeft.stop();
        this.frontRight.stop();
        this.backLeft.stop();
        this.backRight.stop();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putString("SwervePose", this.getPose().toString());
        SmartDashboard.putData("Field", this.field);
        SmartDashboard.putString("Sim SwervePose", this.simOdomotry.toString());
        SmartDashboard.putBoolean("Gyro Connect", this.isGyroConnected());
        SmartDashboard.putNumber("GyroAngle", this.getGyroAngle());
    }
}
