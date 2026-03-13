// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import edu.wpi.first.hal.PWMConfigDataResult;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;



public class R2Jesu_IntakeSubsystem extends SubsystemBase {

  private SparkMax intake_armMotor = new SparkMax(51, MotorType.kBrushless);
  private SparkMax intake_wheelsMotor = new SparkMax(54, MotorType.kBrushless);
  private Encoder intake_armEncoder = new Encoder(0,1, true, CounterBase.EncodingType.k4X);
  private static int targetPosition=0;
  private double intake_armPositions[] = {0.0, 5000.0};
  private PIDController m_armUpController = new PIDController(.00025, 0.0, 0.0, 0.01); //p 1.5
  private PIDController m_armDownController = new PIDController(.00025, 0.0, 0.0, 0.01); //p 1.5
  private double up_pidOutput;
  private double down_pidOutput;
  private PWMConfigDataResult myResult; //no idea
 

 
  /** Creates a new R2Jesu_IntakeSubsystem. */

  public R2Jesu_IntakeSubsystem() {
    // Query some boolean state, such as a digital sensor.
    
  }


  /**
   * R2Jesu_Intake command factory method.
   *
   * @return a command
   */
  public Command R2Jesu_IntakeMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          intake_armEncoder.reset();
        });
  }

   public boolean isIntakeRaised() {
    // If Intake is up, true. 
    if (intake_armEncoder.getDistance() < 1){
       return true;
    }
    else {
      return false;
    }
  }

    // lowers the intake arm to the floor and starts intake
    //  TO DO: add checks to make sure robot is not against a wall
   public void lowerIntake() {
    targetPosition=1;
    intake_wheelsMotor.set(.5);
  } 

  // raise the arm, stop wheels
  //  TO DO: add checks to make sure robot is not against a wall
  public void raiseIntake() {
    targetPosition=0;
    intake_wheelsMotor.set(0);
  }  

  // provided for error condition to stop intake if needed
  public void stopIntakeWheels() {
    intake_wheelsMotor.set(0);
  } 
 
  // moves the intake arm at designated speed
  public void moveArm(double speed) {
    intake_armMotor.set(speed);
  } 
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("encoderdistance", intake_armEncoder.getDistance());
    down_pidOutput = m_armDownController.calculate(intake_armEncoder.getDistance(),intake_armPositions[targetPosition]);
    up_pidOutput = m_armUpController.calculate(intake_armEncoder.getDistance(),intake_armPositions[targetPosition]); 

    if (targetPosition ==0)
    {
        this.moveArm(up_pidOutput);
    }
    else
    {
      this.moveArm(down_pidOutput);
    } 
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
