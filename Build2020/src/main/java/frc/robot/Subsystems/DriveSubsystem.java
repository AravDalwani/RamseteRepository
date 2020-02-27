/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {
  
  private final WPI_TalonSRX FR;
  private final WPI_TalonSRX BR;
  private final WPI_TalonSRX FL;
  private final WPI_TalonSRX BL;  
  
  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    FR = new WPI_TalonSRX(Constants.FR_port);
    BR = new WPI_TalonSRX(Constants.BR_port);
    FL = new WPI_TalonSRX(Constants.FL_port);
    BL = new WPI_TalonSRX(Constants.BL_port);  

    FR.setInverted(true);
    BR.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void drive(double leftSpeed, double rightSpeed) {
    FR.set(rightSpeed);
    BR.set(rightSpeed);
    FL.set(leftSpeed);
    BL.set(leftSpeed);
  }


  double integral_DriveStraight = 0;
  double prev_Error = 0;

  double integral_NavX = 0;
  double prev_Error_NavX = 0;

  public void Drive_Straight(double yaxis) {

    // Encoder PID

    if (Math.abs(yaxis) < Constants.integralResetBound) integral_DriveStraight = 0;

    double shaftLeftRate = RobotContainer.enc_L.getRate();
    double shaftRightRate = RobotContainer.enc_R.getRate();

    SmartDashboard.putNumber("Left Encoder Rate", shaftLeftRate);
    SmartDashboard.putNumber("Right Encoder Rate", shaftRightRate);

    double error = shaftLeftRate - shaftRightRate;
    integral_DriveStraight += (error * Constants.kI_DriveStraight);

    SmartDashboard.putNumber("Error between Encoders", error);

    double derivative = (error - prev_Error) * Constants.kD_DriveStraight;
    prev_Error = error; 

    double correction = (error * Constants.kP_DriveStraight);
    correction += (integral_DriveStraight);
    correction += (derivative);

    // NAVX PID

    if (Math.abs(yaxis) < Constants.integralResetBound) integral_NavX = 0;

    double navxYawAxisRate = RobotContainer.navx.getRate();
    SmartDashboard.putNumber("Yaw Rate", navxYawAxisRate);

    double error_NavX = navxYawAxisRate;
    integral_DriveStraight += (navxYawAxisRate * Constants.kI_NavX);

    double derivative_NavX = (error_NavX - prev_Error_NavX) * Constants.kD_NavX;
    prev_Error_NavX = error_NavX; 

    double correction_NavX = (error_NavX * Constants.kP_NavX);
    correction_NavX += (integral_NavX);
    correction_NavX += (derivative_NavX);

    double left = yaxis - correction - correction_NavX;
    double right = yaxis + correction + correction_NavX;

    drive(yaxis*Constants.maxSpeed, yaxis*Constants.maxSpeed);

  }



  public void Drive_Steer(double yaxis, double zaxis) {
    
    double shaftLeftRate = RobotContainer.enc_L.getRate();
    double shaftRightRate = RobotContainer.enc_R.getRate();
    
    SmartDashboard.putNumber("Left Encoder Rate", shaftLeftRate);
    SmartDashboard.putNumber("Right Encoder Rate", shaftRightRate);

    double error = zaxis*Constants.swerveCoefficient;

    SmartDashboard.putNumber("Error between encoders", Math.abs(shaftLeftRate+shaftRightRate));

    double left = yaxis + error;
    double right = yaxis - error;
    drive(left*Constants.maxSpeed, right*Constants.maxSpeed);

  }


  double integral_PointTurn = 0;

  public void Drive_Turn(double zaxis) {

    if (Math.abs(zaxis) < Constants.integralResetBound) integral_PointTurn = 0;
    
    double shaftLeftRate = RobotContainer.enc_L.getRate();
    double shaftRightRate = RobotContainer.enc_R.getRate();

    SmartDashboard.putNumber("Left Encoder Rate", shaftLeftRate);
    SmartDashboard.putNumber("Right Encoder Rate", shaftRightRate);
    
    double error = shaftLeftRate + shaftRightRate;
    integral_PointTurn += error;

    double correction = (error * Constants.kP_DriveTurn);
    correction += (integral_PointTurn * Constants.kI_DriveTurn);

    double left = (zaxis + correction);
    double right = -1 * (zaxis - correction);

    drive(zaxis*Constants.maxSpeed, -zaxis*Constants.maxSpeed);
    
  }




}
