/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveCommands.DriveCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.ControlPanelCommands.PositionCommand;
import frc.robot.ControlPanelCommands.RotationCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.DriveCommands.SteerCommand;
import frc.robot.SolenoidCommands.UpCommand;
import frc.robot.SolenoidCommands.DownCommand;
import frc.robot.Subsystems.ControlPanelSubsystem;
import frc.robot.Subsystems.DifferentialDriveSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.SolenoidSubsystem;
import frc.robot.Subsystems.TrajectorySubsystem;
import java.util.List;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  // Joystick kept public
  public static Joystick joy1 = new Joystick(1);
  public static Encoder enc_L = new Encoder(0, 1, true, Encoder.EncodingType.k4X);
  public static Encoder enc_R = new Encoder(2, 3, false, Encoder.EncodingType.k4X);
  public static AHRS navx = new AHRS(SPI.Port.kMXP);

  private final DifferentialDriveSubsystem driveSubsystem2 = new DifferentialDriveSubsystem(navx);
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final TrajectorySubsystem trajectorySubsystem = new TrajectorySubsystem();
  private final ControlPanelSubsystem controlPanelSubsystem = new ControlPanelSubsystem();
  private final SolenoidSubsystem solenoidSubsystem = new SolenoidSubsystem();
  
  private final DriveCommand driveCommand = new DriveCommand(driveSubsystem);
  private final SteerCommand steerCommand = new SteerCommand(driveSubsystem2);
  private final UpCommand upCommand = new UpCommand(solenoidSubsystem);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    driveSubsystem2.setDefaultCommand(steerCommand);
    solenoidSubsystem.setDefaultCommand(upCommand);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
   
    JoystickButton commandBrakeButton = new JoystickButton(joy1, Constants.brakeButtonNumber);
    // JoystickButton commandSteerButton = new JoystickButton(joy1, Constants.steerButtonNumber);
    JoystickButton rotationControlButton = new JoystickButton(joy1, Constants.rotationButtonNumber);
    JoystickButton positionControlButton = new JoystickButton(joy1, Constants.positionButtonNumber);
    JoystickButton solenoidButton = new JoystickButton(joy1, Constants.solenoidButtonNumber);
    
    // commandBrakeButton.whenPressed(new BrakeCommand(driveSubsystem));
    // commandSteerButton.whenPressed(new SteerCommand(driveSubsystem));

    // rotationControlButton.whenPressed(new RotationCommand(controlPanelSubsystem));  
    // positionControlButton.whenPressed(new PositionCommand(controlPanelSubsystem));

    rotationControlButton.toggleWhenPressed(new RotationCommand(controlPanelSubsystem));
    positionControlButton.toggleWhenPressed(new PositionCommand(controlPanelSubsystem));

    solenoidButton.toggleWhenPressed(new DownCommand(solenoidSubsystem));    

  }


  public static double getY(final Joystick joy, final double band) {
    // Inverted (Joystick moved forwards gives negtive reading)
    double val = -joy.getY();

    if (Math.abs(val) < band)
        val = 0;
    else {
        val = val - Math.signum(val) * band;
    }
    return val;
  }

  public static double getZ(Joystick joy, double band) {
    double val = joy.getZ();

    if (Math.abs(val) < band)
        val = 0;
    else {
        val = val - Math.signum(val) * band;
    }
    return val;
  }

  public static double getX(Joystick joy, double band) {

    double val = joy.getX();

    if (Math.abs(val) < band)
        val = 0;
    else {
        val = val - Math.signum(val) * band;
    }
    return val;
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    TrajectorySubsystem.resetEncoders();

    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter), DriveConstants.kDriveKinematics, 10);

    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setReversed(true)
        .setKinematics(DriveConstants.kDriveKinematics)
        .addConstraint(autoVoltageConstraint);
    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(Math.PI)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
        //ends at 3 meters after start point
        new Pose2d(3, 0, new Rotation2d(Math.PI)),
        // Pass config
        config
    );
    

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        trajectorySubsystem::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        trajectorySubsystem::getWheelSpeeds,

        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),

        trajectorySubsystem::tankDriveVolts,
        trajectorySubsystem 
      );
    
    return ramseteCommand.andThen(() -> trajectorySubsystem.tankDriveVolts(0, 0));
  }
}
