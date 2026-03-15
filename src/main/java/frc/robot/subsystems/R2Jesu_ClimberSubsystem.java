// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.lang.management.ClassLoadingMXBean;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.hal.PWMConfigDataResult;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.subsystems.R2Jesu_IntakeSubsystem;
import com.revrobotics.RelativeEncoder;



public class R2Jesu_ClimberSubsystem extends SubsystemBase {
  
  private final R2Jesu_IntakeSubsystem m_intakeSubsystem;
 
  private SparkMax climbMotor = new SparkMax(55, MotorType.kBrushless);
 // private Encoder climbEncoder = new Encoder(0,1, true, CounterBase.EncodingType.k4X);
  private static int targetPosition=0;
  private double climbPositions[] = {5000.0, 0.0, 4000.0}; //raise hand, climb up, climb down
  private PIDController m_climbUpController = new PIDController(.00025, 0.0, 0.0, 0.01); //p 1.5
  private PIDController m_climbDownController = new PIDController(.00025, 0.0, 0.0, 0.01); //p 1.5
  private double up_pidOutput;
  private double down_pidOutput;
  private double raise_pidOutput;
  private PWMConfigDataResult myResult; //no idea

   // Get the internal encoder object from the motor controller
  private final RelativeEncoder climbEncoder = climbMotor.getEncoder();

  /** Creates a new R2Jesu_ClimberSubsystem. */

  public R2Jesu_ClimberSubsystem(R2Jesu_IntakeSubsystem intake) {
    // Query some boolean state, such as a digital sensor.
    this.m_intakeSubsystem = intake;
  }

  /**
   * R2Jesu_Climber command factory method.
   *
   * @return a command
   */
  public Command R2Jesu_ClimberMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
        });
  }

/*   public void runClimber(double speed) {
          //if the intake is down, raise it before climbing
          if(!(m_intakeSubsystem.isIntakeRaised())){
            m_intakeSubsystem.raiseIntake();
          }    
  } 
 */

// moves the climber at designated speed, called from periodic until meets target

  public void moveClimber(double speed) {
            climbMotor.set(speed);
  } 

  // put hand in the air and raise intake if needed before moving into position to climb, hand all the way up
   public void raiseHand() {
      targetPosition=0;
      if(!(m_intakeSubsystem.isIntakeRaised())){
        m_intakeSubsystem.raiseIntake();
       }
      climbMotor.set(.1);      
  }

  // lower the robot back down to the floor, hand all the way up
   public void climbDown() {
      targetPosition=2;
      climbMotor.set(.5);
  } 

// Pull the arm down to raise the robot off the floor, hand all the way retracted
  public void climbUp() {
      targetPosition=1;
      climbMotor.set(.5);
  }  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("encoderdistance", hangerEncoder.getDistance());
    SmartDashboard.putNumber("Climbdistance", climbEncoder.getPosition());
    down_pidOutput = m_climbUpController.calculate(climbEncoder.getPosition(),climbPositions[targetPosition]);
    up_pidOutput = m_climbDownController.calculate(climbEncoder.getPosition(),climbPositions[targetPosition]); 
    raise_pidOutput = m_climbDownController.calculate(climbEncoder.getPosition(),climbPositions[targetPosition]); 

    if (targetPosition ==0)
    {
        this.moveClimber(raise_pidOutput);
    }    
    else if (targetPosition ==1)
    {
      this.moveClimber(up_pidOutput);
    } 
   else if (targetPosition ==2)
    {
      this.moveClimber(down_pidOutput);
    } 
   
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
