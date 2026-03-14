// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands;


import frc.robot.subsystems.R2Jesu_IntakeSubsystem; //replace with the subsytem(s) needed for your command
//import frc.robot.subsystems.DriveSubsystem; //replace with the subsytem(s) needed for your command
import edu.wpi.first.wpilibj2.command.Command;


/** An example command that uses an example subsystem. */
public class R2Jesu_RaiseIntakeCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final R2Jesu_IntakeSubsystem m_intakeSubsystem;
    /*  Lowers the intake and starts
     * @param subsystem The subsystem used by this command.
     */
    public R2Jesu_RaiseIntakeCommand(R2Jesu_IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        // Use addRequirements() here to declare subsystem dependencies for each subsytem used
        addRequirements(intakeSubsystem);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_intakeSubsystem.raiseIntake();
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
