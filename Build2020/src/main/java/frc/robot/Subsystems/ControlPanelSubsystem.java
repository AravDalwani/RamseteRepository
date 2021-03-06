/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ControlPanelSubsystem extends SubsystemBase {
  /**
   * Creates a new ControlPanelSubsystem.
   */

  private final VictorSP CP;
  
  public ControlPanelSubsystem() {
    CP = new VictorSP(Constants.CP_port);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed){
    CP.set(speed);
  }

}
