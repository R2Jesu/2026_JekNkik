// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.R2Jesu_ShooterSubsystem;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.*;



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
  private int m_stateMachine = 1;
  private double m_distanceToAprilTag = 0;
  private double m_angleToAprilTag = 0;
  private double m_currentRobotHeading = 0; 
  private double m_newAngleHeading = 0;
  private int m_debounceCounter = 0;
  private int m_debounceLimit = 0;
  private double m_limeLightToAprilTagVerticalDistance = (Constants.kAprilTagHeight - Constants.kLimelightHeight);
  private double m_verticalAngleToAprilTag = 0;
  private CommandXboxController m_joystick;

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
   
    //Initialize State Machine
    m_stateMachine = 1;

    m_debounceCounter = 0;
    m_debounceLimit = 10;
  
    m_isFinished = false;

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
        }

        m_drivetrain.setControl(m_PIDAim.withVelocityX(yLimiter.calculate(-m_joystick.getRightY()))
            .withVelocityY(xLimiter.calculate(-m_joystick.getRightX()))
            .withRotationalRate(-pid.calculate(m_drivetrain.getState().RawHeading.getDegrees(), m_newAngleHeading) * (1.5 * Math.PI)));
        if (Math.abs(pid.getPositionError()) <= Constants.kTXTolerance) {
          if (m_debounceCounter >= m_debounceLimit) {
            m_stateMachine = m_stateMachine + 1;
          } else {
            m_debounceCounter++;
          }
        }
        else {
          m_debounceCounter = 0;
        }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Command Operator ShooerModeShootWithPose: Finished");
    System.out.println("==========================");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    LimelightHelpers.SetIMUAssistAlpha(Constants.kLimelightName, .001);
    return m_isFinished;
  }
}