package frc.robot.lib.math;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.util.FastMath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class MathHelper {
    public static Pose2d absToRelPose(Pose2d pose2d) {
        if (DriverStation.getAlliance().get() == Alliance.Blue) return pose2d;
        Translation2d pos = pose2d.getTranslation().unaryMinus().plus(new Translation2d(17.55, 8.05));
        Rotation2d rot = pose2d.getRotation().rotateBy(new Rotation2d(Math.PI));
        return new Pose2d(pos, rot);
    }

    public static double capPeriodic(double a, double period) {
        a = a % period;
        if (a > period / 2.0) return a - period;
        else if (a <= -period / 2.0) return a + period;
        else return a;
    }

    public static double applyMax(double a, double max) {
        return FastMath.min(max, FastMath.abs(a)) * getSign(a);
    }
    
    public static double getSign(double a) {
        return (a == 0) ? 0 : (a / FastMath.abs(a));
    }
    
    public static Translation2d rotation2dMatrix(Translation2d vector, double theta) {
        RealVector realVector = new ArrayRealVector(new double[]{vector.getX(), vector.getY()});
        double[][] matrixData = {
            {Math.cos(theta), -Math.sin(theta)},
            {Math.sin(theta), Math.cos(theta)}
        };
        RealMatrix rotationMatrix = MatrixUtils.createRealMatrix(matrixData);
        RealVector rotatedVector = rotationMatrix.operate(realVector);
        return new Translation2d(rotatedVector.getEntry(0), rotatedVector.getEntry(1));
    }

    public static boolean isArrivePose(Pose2d currPose, Pose2d targetPose, double normTolerance) {
        Translation2d vector = targetPose.getTranslation().minus(currPose.getTranslation());
        return vector.getNorm() <= normTolerance;
    }
}
