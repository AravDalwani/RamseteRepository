/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// import java.util.ArrayList;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.TrajectorySubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after crea
 * ting this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  Command autonomousCommand;
  RobotContainer robotContainer;
  
  // public static ArrayList<String> colors = new ArrayList<>();

  public static int count = 0;

  public static boolean controlPanelShutdown = false;

  public static String p_color = "";

  public static double red = 0.0;
  public static double green = 0.0;    
  public static double blue = 0.0;

  public static String color = "";
  
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    SmartDashboard.putNumber("P", Constants.kP_NavX);
    SmartDashboard.putNumber("I", Constants.kI_NavX);
    SmartDashboard.putNumber("D", Constants.kD_NavX);
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();


    RawColor detectedColor = colorSensor.getRawColor();
    double IR = colorSensor.getIR();

    double red = detectedColor.red;
    double green = detectedColor.green;
    double blue = detectedColor.blue;
    double redgreen = red/green;
    double greenblue = green/blue;
    double redblue = red/blue;
    
    String color = colorDetection(red, green, blue);
  
    SmartDashboard.putNumber("Red", red);
    SmartDashboard.putNumber("Green", green);
    SmartDashboard.putNumber("Blue", blue);
    SmartDashboard.putNumber("Red by Green", redgreen);
    SmartDashboard.putNumber("Green by Blue", greenblue);
    SmartDashboard.putNumber("Red by Blue", redblue);
    SmartDashboard.putNumber("IR", IR);
    SmartDashboard.putString("Color", color);

    SmartDashboard.putString("Game Data", DriverStation.getInstance().getGameSpecificMessage());

    SmartDashboard.putNumber("Distance", TrajectorySubsystem.getAverageEncoderDistance());


  }

  public String colorDetection(double red, double green, double blue) {
    
    double redgreen = red/green;
    double redblue = red/blue;
    double greenblue = green/blue;

    if(redgreen>=0.5 && redgreen<=0.65 && greenblue>=4.1 && greenblue<=5.6){
        return "Y";
    }
    else if(redblue>=0.25 && redblue<=0.7 && greenblue>=1 && greenblue<=1.6){
        return "B";
    }
    else if(redblue>=1 && redblue<=1.2 && redgreen>=0.275 && redgreen<=0.5){
        return "G";
    }
    else if(redblue>=1.5 && redblue<=4.9 && redgreen>=0.75 && redgreen<=1.6){
        return "R";
    }
    else{
        return "ERROR";
    }

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    RobotContainer.navx.reset();
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
