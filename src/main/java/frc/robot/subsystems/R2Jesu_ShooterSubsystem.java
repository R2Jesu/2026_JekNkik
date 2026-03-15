// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.Map;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class R2Jesu_ShooterSubsystem extends SubsystemBase {
 private SparkMax shooterMotor = new SparkMax(52, MotorType.kBrushless);
 private SparkMax kickerbarMotor = new SparkMax(53, MotorType.kBrushed);
  /** Creates a new R2Jesu_ShooterSubsystem. */
 private final SparkClosedLoopController shooterController = shooterMotor.getClosedLoopController();


  public R2Jesu_ShooterSubsystem() {
    // Query some boolean state, such as a digital sensor.
    SparkMaxConfig shooterConfig=new SparkMaxConfig();
    shooterConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

    shooterConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(.0002)
    .i(0.0)
    .d(0.0)
    .outputRange(-1, 1);

    shooterMotor.configure(shooterConfig, 
    ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }


  /**
   * R2Jesu_Shooter command factory method.
   *
   * @return a command
   */
  public Command R2Jesu_ShooterMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          shooterController.setSetpoint(3000, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
          /* one-time action goes here */
        });
  }

  

  public void runShooter(double speed) {
    shooterController.setSetpoint(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    if(speed > 0.0) {
      kickerbarMotor.set(-1.0);
    } else {
      kickerbarMotor.set(0.0);
    }
  }

 

  @Override
  public void periodic() {
   
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
