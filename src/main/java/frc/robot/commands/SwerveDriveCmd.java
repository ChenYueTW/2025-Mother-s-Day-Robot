package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class SwerveDriveCmd extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final Supplier<Double> xSpeed, ySpeed, rotation;
    private final Supplier<Boolean> robotMode, isTrackLeftTag, isTrackRightTag;
    private boolean arrivedAtTag = false;

    public SwerveDriveCmd(
            SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem,
            Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rotation,
            Supplier<Boolean> robotMode, Supplier<Boolean> isTrackLeftTag, Supplier<Boolean> isTrackRightTag) {
        this.swerveSubsystem = swerveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotation = rotation;
        this.robotMode = robotMode;
        this.isTrackLeftTag = isTrackLeftTag;
        this.isTrackRightTag = isTrackRightTag;
        this.addRequirements(this.swerveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Pose2d pose = this.visionSubsystem.getAprilTagFieldPoseFromLastUpdate();

        if (pose != null &&
            (this.isTrackLeftTag.get() || this.isTrackRightTag.get() &&
            !arrivedAtTag)
        ) {
            Translation2d targetVector = pose.getTranslation().plus(
                    new Translation2d(0.54, 0.14 * (this.isTrackLeftTag.get() ? -1.0 : 1.0))
                        .rotateBy(pose.getRotation()));
            Pose2d pose2d = new Pose2d(targetVector, pose.getRotation());
            arrivedAtTag = this.swerveSubsystem.situateRobot(pose2d);
        } else {
            this.swerveSubsystem.driveSwerve(
                    this.xSpeed.get() * (this.robotMode.get() ? -1.0 : 1.0),
                    this.ySpeed.get() * (this.robotMode.get() ? -1.0 : 1.0), this.rotation.get(),
                    (this.robotMode.get() ? false : true));
            this.arrivedAtTag = false;
        }

    }

    @Override
    public void end(boolean interrupted) {
        this.swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
