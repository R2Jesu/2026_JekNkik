package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
  public static final double kLimelightHeight = 25;
  public static final int kAprilTagHeight = 41;
  public static String kLimelightName = "limelight";
  public static final double kTXTolerance = 4;
  public static final double kDefaultShootSpeed = 1500.0;
  public static final PathConstraints teleopConstraints = new PathConstraints(2.12, 1.8, 45, 75);
  public static final Pose2d kRightHang = new Pose2d(1.15, 5.131, Rotation2d.fromDegrees(180));
  public static final Pose2d kLeftHang = new Pose2d(1.785, 3.880, Rotation2d.fromDegrees(0));
  public static final Pose2d moveit = new Pose2d(2.70, 1.77, Rotation2d.fromDegrees(0));

  public static final double default_armSpeed = .25;
  public static final double default_intakeSpeed = .75;
  public static final double kJoystickDeadband = .1;
}
