// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utilities.LimelightHelpers;

/** An SwerveSubsystem command that uses an SwerveSubsystem subsystem. */
public class R2Jesu_SavePositionCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private boolean sideL;
  private Timer dontSeeTagTimer, stopTimer, overallTimer;
  private PIDController xControl = new PIDController(1.5, 0, 0);
  private PIDController yControl = new PIDController(2, 0, 0);  
  private PIDController zControl = new PIDController(.068, 0, .0);

  private final DriveSubsystem m_subsystem;

  /**
   * Creates a new SwerveCommand.
   * 
   * @param subsystem The subsystem used by this command.
   */
  public R2Jesu_SavePositionCommand(DriveSubsystem subsystem) {
    m_subsystem = subsystem; 
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
//     m_subsystem.getOurPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
