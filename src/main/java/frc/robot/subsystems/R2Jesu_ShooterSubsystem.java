// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class R2Jesu_ShooterSubsystem extends SubsystemBase {
  private SparkMax shooterMotor = new SparkMax(52, MotorType.kBrushless);
  private SparkMax kickerbarMotor = new SparkMax(53, MotorType.kBrushed);
  private SparkMax kickerbar2Motor = new SparkMax(55, MotorType.kBrushless);

  /** Creates a new R2Jesu_ShooterSubsystem. */
  private final SparkClosedLoopController shooterController =
      shooterMotor.getClosedLoopController();

  public R2Jesu_ShooterSubsystem() {
    // Query some boolean state, such as a digital sensor.
    SparkMaxConfig shooterConfig = new SparkMaxConfig();
    shooterConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

    shooterConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(.001)
        .i(0.0000007)
        .d(0.00001)
        .outputRange(0, 1);

    shooterMotor.configure(
        shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

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
          // shooterController.setSetpoint(3000, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
          /* one-time action goes here */
        });
  }

  public void runShooter(double speed) {
    if (speed == 0.0) {
      shooterController.setSetpoint(speed, ControlType.kDutyCycle);
    } else{
      shooterController.setSetpoint(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }
/*     if (speed > 0.0) {
      kickerbarMotor.set(-1.0);
      kickerbar2Motor.set(.20);
    } else {
      kickerbarMotor.set(0.0);
      kickerbar2Motor.set(0.0);
    } */
    SmartDashboard.putNumber("Subsystem shoot speed", speed);
  }

  @Override
  public void periodic() {
    if ((shooterController.getSetpoint() > 0.0  
        && (shooterMotor.getEncoder().getVelocity() >= (shooterController.getSetpoint() * .80)))) {
      kickerbarMotor.set(-1.0);
      kickerbar2Motor.set(.10);
    } else {
      kickerbarMotor.set(0.0);
      kickerbar2Motor.set(0.0);
    }
    SmartDashboard.putNumber("Shoot setpoint", shooterController.getSetpoint());
    SmartDashboard.putNumber("Actual Speed", shooterMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Current", shooterMotor.getOutputCurrent());
    SmartDashboard.putNumber("BusVolt", shooterMotor.getBusVoltage());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
