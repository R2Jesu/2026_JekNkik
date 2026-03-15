// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.networktables.GenericEntry;
import java.util.Map;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// use the revrobotics encoder in a closed loop instead of two PIDs
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;


public class R2Jesu_IntakeSubsystem extends SubsystemBase {

  private SparkMax intake_armMotor = new SparkMax(51, MotorType.kBrushless);
  private SparkMax intake_wheelsMotor = new SparkMax(54, MotorType.kBrushless);
  private Encoder intake_armEncoder = new Encoder(0,1, true, CounterBase.EncodingType.k4X);

// use the revrobotics encoder in a closed loop for the wheels
  private final SparkClosedLoopController intake_wheelController = intake_wheelsMotor.getClosedLoopController();
  // feed-forward for wheel velocity controller
  // default wheel RPM to use when lowering intake (change to your robot's target)
  private static final double intake_lower_wheel_rpm = 4500.0;
  // Shuffleboard entry for a number slider to let user change intake RPM (0-9000)
  private GenericEntry intakeRpmEntry;
 // added to use closed loop controller
  
  private static int targetPosition=0;
  private double intake_armPositions[] = {0.0, 5300.0};
  private PIDController m_armUpController = new PIDController(.00025, 0.0, 0.0, 0.01); //p 1.5
  private PIDController m_armDownController = new PIDController(.00025, 0.0, 0.0, 0.01); //p 1.5
  private double up_pidOutput;
  private double down_pidOutput;
  private double rpm=0;
 

 
  /** Creates a new R2Jesu_IntakeSubsystem. */

  public R2Jesu_IntakeSubsystem() {
    // constructor
    SparkMaxConfig wheelsConfig=new SparkMaxConfig();
    wheelsConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

    wheelsConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(.0002)
    .i(0.0)
    .d(0.0)
    .outputRange(-1, 1);

    intake_wheelsMotor.configure(wheelsConfig, 
    ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  // Create a Shuffleboard number slider on the "Intake" tab for runtime tuning.
  // Range 0..9000, initial value from intake_lower_wheel_rpm.
  intakeRpmEntry = Shuffleboard.getTab("Intake")
    .add("Intake RPM", intake_lower_wheel_rpm)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 6000))
    .getEntry();
 
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
    if (intake_armEncoder.getDistance() < 100){
       return true;
    }
    else {
      return false;
    }
  }

    // lowers the intake arm to the floor and starts intake
    //  TO DO: add checks to make sure robot is not against a wall
   public void lowerIntake() {
    if(isIntakeRaised())
    {
      targetPosition=1;
    }
  } 

  // raise the arm, stop wheels
  //  TO DO: add checks to make sure robot is not against a wall
  public void raiseIntake() {
    if(intake_armEncoder.getDistance() < 1){
    }
    else{
      targetPosition=0;
    }
  }  

  // provided for error condition to stop intake if needed
  public void stopIntakeWheels() {
      setIntakeWheelsVelocity(0.0);
  } 
 
  // moves the intake arm at designated speed
  public void moveArm(double speed) {
      intake_armMotor.set(speed);
  } 

  /**
   * Set intake wheel target velocity (RPM) using the closed-loop controller.
   * This uses the SparkClosedLoopController.setReference API.
   * Confirm ControlType and ArbFFUnits names if your REV library version differs.
   */
  public void setIntakeWheelsVelocity(double rpm) {
    // attempt to set closed-loop velocity reference. If your REV API
    // has a different ControlType name, update it accordingly.
    // Log the requested setpoint for debugging
    SmartDashboard.putNumber("intakeWheelTargetRpm", rpm);

    intake_wheelController.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("encoderdistance", intake_armEncoder.getDistance());
    SmartDashboard.putBoolean("isIntakeRaised", isIntakeRaised());
    down_pidOutput = m_armDownController.calculate(intake_armEncoder.getDistance(),intake_armPositions[targetPosition]);
    up_pidOutput = m_armUpController.calculate(intake_armEncoder.getDistance(),intake_armPositions[targetPosition]); 
    rpm = intakeRpmEntry != null ? intakeRpmEntry.getDouble(intake_lower_wheel_rpm) : intake_lower_wheel_rpm;
          

    if (targetPosition ==0)
    {
        this.moveArm(up_pidOutput);
        setIntakeWheelsVelocity(0.0);
    }
    else
    {
      this.moveArm(down_pidOutput);
      setIntakeWheelsVelocity(rpm);
    } 
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
