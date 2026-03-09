// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
// MEE according to Phoenix6 examples this is needed for  warning on pathplanner warmup
import edu.wpi.first.wpilibj2.command.CommandScheduler;



import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.R2Jesu_ShooterSubsystem;
import frc.robot.commands.R2Jesu_ShooterModeShootWithLimelight;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed set to 1 initially - press fn f12 to see setting
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity - change to rotate faster

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors - can change to velocity 
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SendableChooser<Command> autoChooser;

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final DriveSubsystem m_robotDrive = TunerConstants.createDrivetrain();
    public final R2Jesu_ShooterSubsystem m_shooterSubsystem = new R2Jesu_ShooterSubsystem();
    // need to understand why drivetain doesnt = new DriveSubsystem();

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(5.0); // 3 m/s^2
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(5.0);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(Math.PI); // rad/s^2

    public RobotContainer() {
        registerAutoCommands();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        configureBindings();
        
        // MEE ADDED Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

    }

    private void registerAutoCommands(){

  NamedCommands.registerCommand("ppShoot", Commands.print("Command to shoot preloaded balls"));
  NamedCommands.registerCommand("ppHang", new SequentialCommandGroup(Commands.print("Command to hang"), Commands.waitSeconds(1), Commands.print("Command to release")));
  NamedCommands.registerCommand("score", Commands.print("Command to shoot preloaded balls"));
  NamedCommands.registerCommand("hang", new SequentialCommandGroup(Commands.print("Raise arm"), Commands.waitSeconds(1), Commands.print("Drive forward x amount of seconds"), Commands.waitSeconds(1), Commands.print("Pull robot up"), Commands.waitSeconds(5), Commands.print("Release")));
  NamedCommands.registerCommand("intake", new SequentialCommandGroup(Commands.print("Pull down intake"), Commands.waitSeconds(1), Commands.print("Drive forward for x seconds")));
  NamedCommands.registerCommand("throw", Commands.print("Command to shoot preloaded balls x meters"));

    
  }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_robotDrive.setDefaultCommand(
            // m_robotDrive will execute this command periodically
            m_robotDrive.applyRequest(() ->
                drive.withVelocityX(yLimiter.calculate(-joystick.getRightY())) // Drive forward with negative Y (forward)
                    .withVelocityY(xLimiter.calculate(-joystick.getRightX())) // Drive left with negative X (left)
                    //.withRotationalRate(rotLimiter.calculate(-joystick.getLeftX())) // Drive counterclockwise with negative X (left)
                    .withRotationalRate(-joystick.getLeftX()) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            m_robotDrive.applyRequest(() -> idle).ignoringDisable(true)
        );
        
        // MEE brake locks wheels into an X-stance to lock it into a position
        //joystick.a().whileTrue(m_robotDrive.applyRequest(() -> brake));
        //joystick.b().whileTrue(m_robotDrive.applyRequest(() ->
          //  point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
       // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        /* joystick.back().and(joystick.y()).whileTrue(m_robotDrive.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(m_robotDrive.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(m_robotDrive.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(m_robotDrive.sysIdQuasistatic(Direction.kReverse));
        */
        // MEE reset the field-centric heading on left bumper press, redefines what is forward TEST ME
        joystick.leftBumper().onTrue(m_robotDrive.runOnce(() -> m_robotDrive.seedFieldCentric()));

        m_robotDrive.registerTelemetry(logger::telemeterize);

        //R2JESU Driver Buttons and such
        joystick.rightTrigger().whileTrue(new R2Jesu_ShooterModeShootWithLimelight(m_shooterSubsystem, m_robotDrive,
            joystick));

        joystick.button(1).onTrue(
            new ConditionalCommand(
            AutoBuilder.pathfindToPose(Constants.kLeftHang, Constants.teleopConstraints),
            AutoBuilder.pathfindToPose(Constants.kRightHang, Constants.teleopConstraints),
            () -> m_robotDrive.getState().Pose.getY() < 2.94
            )
        );

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
