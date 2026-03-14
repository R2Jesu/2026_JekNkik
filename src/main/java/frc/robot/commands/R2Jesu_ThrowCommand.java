// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands;


import frc.robot.Constants;
import frc.robot.subsystems.R2Jesu_ShooterSubsystem; //replace with the subsytem(s) needed for your command

import java.io.Console;
import java.lang.constant.Constable;

import edu.wpi.first.wpilibj2.command.Command;


/** An example command that uses an example subsystem. */
public class R2Jesu_ThrowCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final R2Jesu_ShooterSubsystem m_shooterSubsystem;
// Throws balls farther up or down the field.

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public R2Jesu_ThrowCommand(R2Jesu_ShooterSubsystem subsystem) {
        m_shooterSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies for each subsytem used
        addRequirements(subsystem);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println("Execute");
        m_shooterSubsystem.runShooter(Constants.kDefaultShootSpeed);
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("end");
        m_shooterSubsystem.runShooter(0.0);
    }


    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
