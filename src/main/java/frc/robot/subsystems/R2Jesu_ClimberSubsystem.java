// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.hal.PWMConfigDataResult;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class R2Jesu_ClimberSubsystem extends SubsystemBase {

  private final R2Jesu_IntakeSubsystem m_intakeSubsystem;

  private SparkMax climbMotor = new SparkMax(55, MotorType.kBrushless);
  private static int targetPosition = 0;
  private double climbPositions[] = {
    -300.0, 0.0, 4000.0, 5000.0
  }; // raise hand, climb up, climb down, retract hand
  private PIDController m_climbUpController = new PIDController(.00025, 0.0, 0.0, 0.01); // p 1.5
  private PIDController m_climbDownController = new PIDController(.00025, 0.0, 0.0, 0.01); // p 1.5
  private PIDController m_noWeightController = new PIDController(.00025, 0.0, 0.0, 0.01); // p 1.5
  private double pidOutput;
  private PWMConfigDataResult myResult; // no idea

  // Get the internal encoder object from the motor controller
  private final RelativeEncoder climbEncoder = climbMotor.getEncoder();
  private SparkMaxConfig climbConfig = new SparkMaxConfig();

  /** Creates a new R2Jesu_ClimberSubsystem. */
  public R2Jesu_ClimberSubsystem(R2Jesu_IntakeSubsystem intake) {
    // Query some boolean state, such as a digital sensor.
    this.m_intakeSubsystem = intake;
    climbConfig.smartCurrentLimit(20);
    climbMotor.configure(
        climbConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * R2Jesu_Climber command factory method.
   *
   * @return a command
   */
  public Command R2Jesu_ClimberMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(() -> {});
  }

  // moves the climber at designated speed, called from periodic until meets target

  public void moveClimber(double speed) {
    climbMotor.set(.5);
  }

  // put hand in the air and raise intake if needed before moving into position to climb, hand all
  // the way up
  public void raiseHand() {
    targetPosition = 0;
    if (!(m_intakeSubsystem.isIntakeRaised())) {
      m_intakeSubsystem.raiseIntake();
    }
  }

  // lower the robot back down to the floor, hand all the way up
  public void climbDown() {
    targetPosition = 2;
  }

  // Pull the arm down to raise the robot off the floor, hand all the way retracted
  public void climbUp() {
    targetPosition = 1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climbdistance", climbEncoder.getPosition());
    if (targetPosition == 0) // assigned in raise hand
    {
      pidOutput =
          -m_noWeightController.calculate(
              climbEncoder.getPosition(), climbPositions[targetPosition]);
    } else if (targetPosition == 1) // assigned in climb up
    {
      pidOutput =
          -m_climbUpController.calculate(
              climbEncoder.getPosition(), climbPositions[targetPosition]);
    } else if (targetPosition == 2) // assigned in climb down
    {
      pidOutput =
          m_climbDownController.calculate(
              climbEncoder.getPosition(), climbPositions[targetPosition]);
    } else if (targetPosition == 3) // assigned in retract hand
    {
      pidOutput =
          m_noWeightController.calculate(
              climbEncoder.getPosition(), -climbPositions[targetPosition]);
    }
    this.moveClimber(pidOutput);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
