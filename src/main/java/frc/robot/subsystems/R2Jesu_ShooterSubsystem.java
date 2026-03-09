// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;




public class R2Jesu_ShooterSubsystem extends SubsystemBase {

 
  /** Creates a new R2Jesu_ShooterSubsystem. */

  public R2Jesu_ShooterSubsystem() {
    // Query some boolean state, such as a digital sensor.
    
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
          /* one-time action goes here */
        });
  }

  

  public void runShooter(double speed) {
 
  } 

 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("encoderdistance", hangerEncoder.getDistance());

   
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
