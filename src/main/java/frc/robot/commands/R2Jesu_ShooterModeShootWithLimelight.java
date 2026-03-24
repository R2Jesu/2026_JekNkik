// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.R2Jesu_ShooterSubsystem;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.LimelightHelpers.PoseEstimate;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/** Aims with limelight */
public class R2Jesu_ShooterModeShootWithLimelight extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final R2Jesu_ShooterSubsystem m_shooterSubsystem;

  private final DriveSubsystem m_drivetrain;

  private final SwerveRequest.FieldCentricFacingAngle m_aim =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDeadband((TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)) * 0.1)
          .withRotationalDeadband((1.5 * Math.PI) * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withVelocityX(0)
          .withVelocityY(0);

  private final SwerveRequest.FieldCentric m_PIDAim =
      new SwerveRequest.FieldCentric()
          .withDeadband((TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)) * 0.1)
          .withRotationalDeadband((1.5 * Math.PI) * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withVelocityX(0)
          .withVelocityY(0);

  private final SlewRateLimiter xLimiter = new SlewRateLimiter(5.0); // 3 m/s^2
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(5.0);

  /*
   * Check for Fiducal 'Whatever it is (7?)'                            CASE 1
   * If present, get TX - If not, quit command (or signal in some way)  CASE 1
   * Add (or subtract) TX as angle, to / from current robot angle       CASE 1
   * That is your setpoint                                              CASE 1
   * Set swerve control the face the setpoint angle
   * Do so until TX is 0, or near 0
   */

  private boolean m_isFinished = false;
  // private double m_distanceToAprilTag = 0;
  private double m_angleToAprilTag = 0;
  private double m_currentRobotHeading = 0;
  private double m_newAngleHeading = 0;
  // private double m_limeLightToAprilTagVerticalDistance = (Constants.kAprilTagHeight -
  // Constants.kLimelightHeight);
  // private double m_verticalAngleToAprilTag = 0;
  private CommandXboxController m_joystick;
  private double m_rotation;
  private List<Double> goodTags = new ArrayList<>();
  private List<Double> adjTags = new ArrayList<>();
  // Distance → RPM lookup table (meters → RPM)
  // private static final double[] kDistances = { 1.5, 2.0, 2.25, 2.5, 3.0, 3.5, 4.0 };
  // private static final double[] kRpms      = { 3750, 4000, 4250, 4500, 4750, 5000, 5500 };
  private static final double[] kRpms = {750, 4000, 4250, 2040, 2040, 800, 5500};
  // default speed to return if none can be calculated
  private double m_defaultVelocity = 1500;
  // Distance -> RPM Calculation Variables
  private double m_xLaunchDistance =
      0; // Horizontal distance from the release point to the center of the hoop.
  private double m_yLaunchHeight =
      (72 - 11.5)
          * 0.0254; // meters-Vertical distance (height difference) between the release point 11.5"
  // and the hoop(72"), constant
  private double m_hoopRadius =
      23.5 * 0.0254; // meters-radius of the target, defined constant 41.17/2
  private double m_gAccelGravity = 9.81; // Acceleration due to gravity approximation m/s2, constant
  private double m_launcherSetback =
      9.5 * 0.0254; // distance in meters that the shooter is set back from the limelight ?5"?
  private double m_tLaunchAngle =
      Math.toRadians(38.9); // Launch angle relative to the horizontal in degrees, constant
  private double m_numerator = 0;
  private double m_denominator = 0;
  private double m_kRpmsCalc = 0;
  private double runVelocity = 0;

  Optional<Alliance> alliance = DriverStation.getAlliance();
  private PoseEstimate pose;

  PIDController pid = new PIDController(.01, 0.00, 0.00);

  private Timer theTimer = new Timer();

  /**
   * Constructs an instance of the aim with limelight command.
   *
   * @param frontIntakeSubsystem An instance of the front intake subsystem. Required.
   * @param shooterSubsystem An instance of the shooter subsystem. Required.
   * @param drivetrain An instance of the drivetrain subsystem. Required.
   */
  public R2Jesu_ShooterModeShootWithLimelight(
      R2Jesu_ShooterSubsystem shooterSubsystem,
      DriveSubsystem drivetrain,
      CommandXboxController theJoystick) {
    m_shooterSubsystem = shooterSubsystem;
    m_drivetrain = drivetrain;
    m_joystick = theJoystick;

    m_aim.HeadingController.setPID(20, 0, 0.05);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterSubsystem);
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    System.out.println("==========================");
    System.out.println("Command Operator: AimWithLimelight");

    m_isFinished = false;
    m_rotation = 0.0;

    // Need to add more tags
    goodTags.add(25.0);
    goodTags.add(26.0);
    goodTags.add(18.0);
    goodTags.add(21.0);
    goodTags.add(24.0);
    goodTags.add(27.0);
    goodTags.add(9.0);
    goodTags.add(10.0);
    goodTags.add(11.0);
    goodTags.add(8.0);
    goodTags.add(5.0);
    goodTags.add(2.0);

    adjTags.add(9.0);
    adjTags.add(10.0);
    adjTags.add(25.0);
    adjTags.add(26.0);

    LimelightHelpers.SetIMUAssistAlpha(Constants.kLimelightName, .01);
    //   double dMeters = pose.avgTagDist + m_hoopRadius + m_launcherSetback;
    // SmartDashboard.putNumber("dmeter", dMeters);
    theTimer.start();
    theTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_isFinished = false;
    theTimer.reset();

    if (LimelightHelpers.getTV(
        Constants
            .kLimelightName)) { // As I understand, the pipeline defines the AprilTag to look for.
      // There may be a way to further refine.
      m_angleToAprilTag = LimelightHelpers.getTX(Constants.kLimelightName);
      m_currentRobotHeading = m_drivetrain.getState().RawHeading.getDegrees();
      m_newAngleHeading = m_angleToAprilTag + m_currentRobotHeading;
      // If we are looking at the offset tags and from the right take a margin off the ajustment to
      // keep it more centered
      // Need to check that a positive angle to tag is correct but I think it all runs
      // counterclockwise.  Need to verify.
      if (adjTags.contains(LimelightHelpers.getFiducialID(Constants.kLimelightName))
          && m_angleToAprilTag < 0) {
        m_newAngleHeading = m_newAngleHeading + 9.0;
      }
      // m_verticalAngleToAprilTag = LimelightHelpers.getTY(Constants.kLimelightName);
      // m_distanceToAprilTag = m_limeLightToAprilTagVerticalDistance /
      // Math.tan(Math.toRadians(m_verticalAngleToAprilTag));
      m_rotation =
          -pid.calculate(m_drivetrain.getState().RawHeading.getDegrees(), m_newAngleHeading)
              * (1.5 * Math.PI);
    } else {
      m_rotation = 0.0;
    }
    SmartDashboard.putNumber("new angle", m_newAngleHeading);

    // Here add if the tag value is not a hub tag also do not rotate
    if (!(goodTags.contains(LimelightHelpers.getFiducialID(Constants.kLimelightName)))) {
      m_rotation = 0.0;
    }

    // Scale joystick inputs to meters/sec so the drivetrain sees real-world speeds
    // (TunerConstants.kSpeedAt12Volts is the theoretical max speed at 12V)
    double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    m_drivetrain.setControl(
        m_PIDAim
            .withVelocityX(yLimiter.calculate(
              MathUtil.applyDeadband(-m_joystick.getRightY(), Constants.kJoystickDeadband) 
              * maxSpeed))
            .withVelocityY(xLimiter.calculate(
              MathUtil.applyDeadband(-m_joystick.getRightX(), Constants.kJoystickDeadband) 
              * maxSpeed))
            .withRotationalRate(m_rotation));

    m_shooterSubsystem.runShooter(rpmForDistance());

    if (theTimer.hasElapsed(5)
        && m_joystick.getRightTriggerAxis() == 0
        && m_joystick.getLeftTriggerAxis() == 0) {
      m_isFinished = true;
      System.out.println("Timer elapsed and exited");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("shooter interrupted");
    m_shooterSubsystem.runShooter(0);
    LimelightHelpers.SetIMUAssistAlpha(Constants.kLimelightName, .001);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }

  private double rpmForDistance() {

    pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.kLimelightName);
    boolean m_shotpossible = true;
    SmartDashboard.putBoolean("ShotPossible", m_shotpossible);
    if (pose == null) {
      System.out.println("Null pose");
      // Limelight didn't return a pose estimate; fall back to a safe default RPM
      m_shotpossible = false;
      SmartDashboard.putNumber("Shoot RPM1", kRpms[0]);
      SmartDashboard.putBoolean(
          "impossibleShot",
          m_shotpossible); // what is this printing out if we are returning default velocity?
      return m_defaultVelocity; // default velocity to return set in variable section
    }
    // avgTagDist is off by 1-4 inches; suspect angles are the problem
    double dMeters = pose.avgTagDist;
    SmartDashboard.putNumber("dmeter", dMeters);
    // Calculate velocity based on projectile motion equation
    // limelight distance plus radius of hoop and setback of launcher from
    m_xLaunchDistance = dMeters + m_hoopRadius + m_launcherSetback;
    SmartDashboard.putNumber("distance", m_xLaunchDistance);

    // g*Xsquared  acceleration gravity * distance to center of target squared
    m_numerator = m_gAccelGravity * Math.pow(m_xLaunchDistance, 2); // g*xsquared measured in meters
    SmartDashboard.putNumber("numerator", m_numerator);

    // 2*cos squared(theta)(x tan(theta)-y measured in radians
    m_denominator =
        2
            * Math.pow(Math.cos(m_tLaunchAngle), 2)
            * (m_xLaunchDistance * Math.tan(m_tLaunchAngle) - m_yLaunchHeight);
    SmartDashboard.putNumber("denominator", m_denominator);

    // make sure shot is physically possible, if not ... do ??? nothing ??? LED light???
    if (m_denominator <= 0) {
      m_shotpossible = false;
      SmartDashboard.putBoolean("impossibleShot", m_shotpossible);
      return runVelocity; // set status light
    } else {
      // this is in m/s - need to convert m/s to rpm by ???velocityMps / 2 * Math.PI *
      // wheelRadiusMeters)*GearRatio
      // m/s is 251 RPM
      m_shotpossible = true;
      double m_factor = 2.0; // multiplier for basketball-fuel conversion
      SmartDashboard.putBoolean("impossibleShot", m_shotpossible);
      m_kRpmsCalc = (Math.sqrt(m_numerator / m_denominator) * 187.97);
      double m_maxVelocity =
          5000.0; // max velocity for motor, if number is greater than this only send the max
      SmartDashboard.putNumber(
          "kRpms Calc", MathUtil.clamp(m_kRpmsCalc * m_factor, 0.0, m_maxVelocity));
      runVelocity =
          MathUtil.clamp(
              m_kRpmsCalc * m_factor,
              0.0,
              m_maxVelocity); // prevents sending impossible value to motors
      return runVelocity;
    }
  }
}
