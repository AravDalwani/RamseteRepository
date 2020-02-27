/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SolenoidSubsystem extends SubsystemBase {

  private final DoubleSolenoid doubleSolenoid;

  /**
   * Creates a new SolenoidSubsystem.
   */
  public SolenoidSubsystem() {
    doubleSolenoid =  new DoubleSolenoid(Constants.SR_port, Constants.SF_port);  
  }

  public void setMode(int mode){
    switch(mode){
      case 1:doubleSolenoid.set(DoubleSolenoid.Value.kForward);
      break;
      case 2:doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
      break;
      case 3:doubleSolenoid.set(DoubleSolenoid.Value.kOff);
      break;
      default:doubleSolenoid.set(DoubleSolenoid.Value.kForward);
      break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
