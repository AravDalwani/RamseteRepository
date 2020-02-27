/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.DriveCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final DriveSubsystem driveSubsystem;
  
  /**
   * Creates a new DriveCommand.
   */
  public DriveCommand(Subsystem driveSubsystem) {
    
    this.driveSubsystem = (DriveSubsystem) driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double yaxis = RobotContainer.getY(RobotContainer.joy1, Constants.yDeadband); // Adjusted Y
    double zaxis = RobotContainer.getZ(RobotContainer.joy1, Constants.zDeadband); // Adjusted Z

    SmartDashboard.putNumber("Y-AXIS", yaxis);
    SmartDashboard.putNumber("Z-AXIS", zaxis);
    
  
    if (Math.abs(zaxis) > Constants.zTurnThreshold) {
      driveSubsystem.Drive_Turn(zaxis);
    } else {
      if (yaxis != 0) driveSubsystem.Drive_Straight(yaxis);
      else driveSubsystem.drive(0, 0);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
