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
import frc.robot.Subsystems.DifferentialDriveSubsystem;

public class SteerCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final DifferentialDriveSubsystem driveSubsystem;
  
  /**
   * Creates a new SteerCommand.
   */
  public SteerCommand(Subsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = (DifferentialDriveSubsystem) driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double yaxis = RobotContainer.getY(RobotContainer.joy1, Constants.yDeadband); //Adjusted Y
    double zaxis = RobotContainer.getZ(RobotContainer.joy1, Constants.zDeadband); // Adjusted Z

    SmartDashboard.putNumber("Y-AXIS", yaxis);
    SmartDashboard.putNumber("Z-AXIS", zaxis);

    driveSubsystem.drive(yaxis, zaxis);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return !RobotContainer.joy1.getRawButton(Constants.steerButtonNumber);
    return false;
  }
}
