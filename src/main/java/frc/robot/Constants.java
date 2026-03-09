package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    public static final double kLimelightHeight = 25;
    public static final int kAprilTagHeight = 41;
    public static String kLimelightName = "limelight";
    public static final double kTXTolerance = 4;
    public static final PathConstraints teleopConstraints = new PathConstraints(
        2.12, 1.8, 45, 75);
    public static final Pose2d kRightHang = new Pose2d(1.02, 4.69, Rotation2d.fromDegrees(180));
    public static final Pose2d kLeftHang = new Pose2d(0.96, 2.94, Rotation2d.fromDegrees(0));
}
