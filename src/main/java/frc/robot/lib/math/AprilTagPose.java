package frc.robot.lib.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public enum AprilTagPose {
        I(657.37, 25.80, 58.50, 126, 0),
        II(657.37, 291.20, 58.50, 234, 0),
        III(455.15, 317.15, 51.25, 270, 0),
        IV(365.20, 241.64, 73.54, 0, 30),
        V(365.20, 75.39, 73.54, 0, 30),
        VI(530.49, 130.17, 12.13, 300, 0),
        VII(546.87, 158.50, 12.13, 0, 0),
        VIII(530.49, 186.83, 12.13, 60, 0),
        IX(497.77, 186.83, 12.13, 120, 0),
        X(481.39, 158.50, 12.13, 180, 0),
        XI(497.77, 130.17, 12.13, 240, 0),
        XII(33.51, 25.80, 58.50, 54, 0),
        XIII(33.51, 291.20, 58.50, 306, 0),
        XIV(325.68, 241.64, 73.54, 180, 30),
        XV(325.68, 75.39, 73.54, 180, 30),
        XVI(235.73, -0.15, 51.25, 90, 0),
        XVII(160.39, 130.17, 12.13, 240, 0),
        XVIII(144.00, 158.50, 12.13, 180, 0),
        XIX(160.39, 186.83, 12.13, 120, 0),
        XX(193.10, 186.83, 12.13, 60, 0),
        XXI(209.49, 158.50, 12.13, 0, 0),
        XXII(193.10, 130.17, 12.13, 300, 0);

    private final double x;
    private final double y;
    private final double z;
    private final double zRot;
    private final double yRot;

    AprilTagPose(double x, double y, double z, double zRot, double yRot) {
        this.x = Units.inchesToMeters(x);
        this.y = Units.inchesToMeters(y);
        this.z = Units.inchesToMeters(z);
        this.zRot = zRot;
        this.yRot = yRot;
    }

    private Pose3d getPose3d() {
        return new Pose3d(this.x, this.y, this.z, new Rotation3d(this.zRot, 0.0, this.yRot));
    }

    private Rotation2d getRotation2d() {
        return new Rotation2d(Units.degreesToRadians(this.zRot));
    }

    public static Pose3d getPose3d(double id) {
        return values()[(int) (id - 1)].getPose3d();
    }

    public static Rotation2d getRotation2d(double id) {
        Rotation2d rot = values()[(int) (id - 1)].getRotation2d();
        if (DriverStation.getAlliance().get() == Alliance.Red) rot = rot.rotateBy(new Rotation2d(Math.PI));
        return rot;
    }

    public static Translation3d getTranslation3d(double id) {
        Pose2d pose = MathHelper.absToRelPose(getPose3d(id).toPose2d());
        return new Translation3d(pose.getX(), pose.getY(), getPose3d(id).getZ());
    }
}
