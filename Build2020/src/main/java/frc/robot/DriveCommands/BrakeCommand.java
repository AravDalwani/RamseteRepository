/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.DriveCommands;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.DifferentialDriveSubsystem;
import frc.robot.Subsystems.DriveSubsystem;

public class BrakeCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final DriveSubsystem driveSubsystem;
  
  /**
   * Creates a new BrakeCommand.
   */
  public BrakeCommand(Subsystem driveSubsystem) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = (DriveSubsystem) driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.drive(0,0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!RobotContainer.joy1.getRawButton(Constants.brakeButtonNumber)) || (RobotContainer.enc_L.getRate() == 0.0 && RobotContainer.enc_R.getRate() == 0.0);
  }
}
