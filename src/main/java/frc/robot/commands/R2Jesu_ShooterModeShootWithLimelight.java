// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.R2Jesu_ShooterSubsystem;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import java.lang.Math;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import static edu.wpi.first.units.Units.*;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;



/**
* Aims with limelight
*/
public class R2Jesu_ShooterModeShootWithLimelight extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final R2Jesu_ShooterSubsystem m_shooterSubsystem;
  private final DriveSubsystem m_drivetrain;

  private final SwerveRequest.FieldCentricFacingAngle m_aim = new SwerveRequest.FieldCentricFacingAngle()
  .withDeadband((TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)) * 0.1)
  .withRotationalDeadband((1.5 * Math.PI) * 0.1)
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
  .withVelocityX(0)
  .withVelocityY(0);

  private final SwerveRequest.FieldCentric m_PIDAim = new SwerveRequest.FieldCentric()
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
  private double m_distanceToAprilTag = 0;
  private double m_angleToAprilTag = 0;
  private double m_currentRobotHeading = 0; 
  private double m_newAngleHeading = 0;
  private double m_limeLightToAprilTagVerticalDistance = (Constants.kAprilTagHeight - Constants.kLimelightHeight);
  private double m_verticalAngleToAprilTag = 0;
  private CommandXboxController m_joystick;
  private double m_rotation;
  private List<Double> goodTags = new ArrayList<>();
  // Distance → RPM lookup table (meters → RPM)
  private static final double[] kDistances = { 1.5, 2.0, 2.25, 2.5, 3.0, 3.5, 4.0 };
  private static final double[] kRpms      = { 3750, 4000, 4250, 4500, 4750, 5000, 5500 };
  // default speed to return if none can be calculated
  private double m_defaultVelocity = 1500;
  // Distance -> RPM Caluclation Variables
  private double m_xLaunchDistance = 0; //Horizontal distance from the release point to the center of the hoop.
  private double m_yLaunchHeight = (72-21)*0.0254; // meters-Vertical distance (height difference) between the release point ?21"? and the hoop(72"), constant
  private double m_hoopRadius = 20.585*0.0254; // meters-radius of the target, defined constant 41.17/2
  private double m_gAccelGravity = 9.81; //Acceleration due to gravity approximation m/s2, constant
  private double m_launcherSetback= 5*0.0254; // distance in meters that the shooter is set back from the limelight ?5"?
  private double m_tLaunchAngle = Math.toRadians(17); // Launch angle relative to the horizontal in degrees, constant
  private double m_numerator = 0;
  private double m_denominator = 0;
  private double m_kRpmsCalc = 0;

  Optional<Alliance> alliance = DriverStation.getAlliance();
  private PoseEstimate pose;

  PIDController pid = new PIDController(.01, 0.00, 0.00);

  /**
   * Constructs an instance of the aim with limelight command.
   * @param frontIntakeSubsystem An instance of the front intake subsystem.
   * Required.
   * @param shooterSubsystem An instance of the shooter subsystem.
   * Required.
   * @param drivetrain An instance of the drivetrain subsystem.
   * Required.
   */
  public R2Jesu_ShooterModeShootWithLimelight(R2Jesu_ShooterSubsystem shooterSubsystem, DriveSubsystem drivetrain,
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

    //Need to add more tags
    goodTags.add(25.0);
    goodTags.add(26.0);
    goodTags.add(18.0);
    goodTags.add(21.0);
    goodTags.add(24.0);
    goodTags.add(27.0);
    goodTags.add(9.0);
    goodTags.add(10.0);

    LimelightHelpers.SetIMUAssistAlpha(Constants.kLimelightName, .01);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (LimelightHelpers.getTV(Constants.kLimelightName)) {//As I understand, the pipeline defines the AprilTag to look for.  There may be a way to further refine.
          m_angleToAprilTag = LimelightHelpers.getTX(Constants.kLimelightName);
          m_currentRobotHeading = m_drivetrain.getState().RawHeading.getDegrees();
          m_newAngleHeading = m_angleToAprilTag + m_currentRobotHeading;
          m_verticalAngleToAprilTag = LimelightHelpers.getTY(Constants.kLimelightName);
          m_distanceToAprilTag = m_limeLightToAprilTagVerticalDistance / Math.tan(Math.toRadians(m_verticalAngleToAprilTag));
          m_rotation = -pid.calculate(m_drivetrain.getState().RawHeading.getDegrees(), m_newAngleHeading) * (1.5 * Math.PI);
        }
        else {
          m_rotation = 0.0;
        }

    //Here add if the tag value is not a hub tag also do not rotate
    if (!(goodTags.contains(LimelightHelpers.getFiducialID(Constants.kLimelightName))))
    {
      m_rotation = 0.0;
    }


  // Scale joystick inputs to meters/sec so the drivetrain sees real-world speeds
  // (TunerConstants.kSpeedAt12Volts is the theoretical max speed at 12V)
  double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  m_drivetrain.setControl(m_PIDAim.withVelocityX(yLimiter.calculate(-m_joystick.getRightY() * maxSpeed))
    .withVelocityY(xLimiter.calculate(-m_joystick.getRightX() * maxSpeed))
    .withRotationalRate(m_rotation));
        
    m_shooterSubsystem.runShooter(rpmForDistance());

  }
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
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
    if (pose == null) {
      // Limelight didn't return a pose estimate; fall back to a safe default RPM
      SmartDashboard.putNumber("Shoot RPM1", kRpms[0]);  // what is this printing out if we are returning default velocity?
      return m_defaultVelocity; // default velocity to return set in variable section
    }
    double dMeters = pose.avgTagDist;
    SmartDashboard.putNumber("dmeter", dMeters);

// Calculate velocity based on projectile motion equation - return statements commented out during testing

    //limelight distance plus radius of hoop and setback of launcher from 
    m_xLaunchDistance = dMeters + m_launcherSetback + m_hoopRadius; 

    // g*Xsquared  acceleration gravity * distance to center of target squared
    m_numerator=m_gAccelGravity*Math.pow(m_xLaunchDistance,2); // g*xsquared measured in meters
    SmartDashboard.putNumber("numerator", m_numerator);

    //2*cos squared(theta)(x tan(theta)-y measured in radians
    m_denominator=2*Math.pow(Math.cos(m_tLaunchAngle),2)*(m_xLaunchDistance*Math.tan(m_tLaunchAngle)-m_yLaunchHeight); 
    SmartDashboard.putNumber("denominator", m_denominator);

    // make sure shot is physically possible, if not ... do ??? nothing ??? LED light???
    if(m_denominator<=0) {
      //return m_defaultVelocity;
      SmartDashboard.putNumber("kRpms Calc",m_defaultVelocity);      
    }
    else {
      // this is in m/s - need to convert m/s to rpm by ???velocityMps / 2 * Math.PI * wheelRadiusMeters)*GearRatio
      // or create helper function
      //     public static double mpsToRps(double velocityMps, double wheelRadiusMeters, double gearRatio) {
      //        double circumference = 2 * Math.PI * wheelRadiusMeters;
      //        double wheelRps = velocityMps / circumference;
      //        return wheelRps * gearRatio;

      m_kRpmsCalc=(Math.sqrt(m_numerator/m_denominator)); // add meters per second conversion 
      double m_maxVelocity=6000.0; // what is the max velocity???
      SmartDashboard.putNumber("kRpms Calc", MathUtil.clamp(m_kRpmsCalc,0.0,m_maxVelocity));      
      //return MathUtil.clamp(m_kRpmsCalc,0.0,m_maxVelocity); // prevents sending impossible value to motors
     }

// Calculate velocity based on fixed array of velocity/distance pairs
    if (dMeters <= kDistances[0]) return kRpms[0];
    if (dMeters >= kDistances[kDistances.length - 1]) return kRpms[kRpms.length - 1];
    for (int i = 0; i < kDistances.length - 1; i++) {
        double d0 = kDistances[i];
        double d1 = kDistances[i + 1];
        if (dMeters >= d0 && dMeters <= d1) {
            double t = (dMeters - d0) / (d1 - d0);
            SmartDashboard.putNumber("Shoot RPM2", kRpms[0]);
            return kRpms[i] + t * (kRpms[i + 1] - kRpms[i]);
        }
    }
    SmartDashboard.putNumber("Shoot RPM3", kRpms[0]);

    return kRpms[0];
    //return 1500;
  }
}