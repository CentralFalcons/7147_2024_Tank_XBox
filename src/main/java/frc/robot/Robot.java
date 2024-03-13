// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

// import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Timer;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {

  private WPI_TalonSRX m_frontLeft = new WPI_TalonSRX(1);
  private WPI_TalonSRX m_rearLeft = new WPI_TalonSRX(2);
  private WPI_TalonSRX m_frontRight = new WPI_TalonSRX(3);
  private WPI_TalonSRX m_rearRight = new WPI_TalonSRX(4);

  private MotorController m_shooterFront = new WPI_VictorSPX(5);
  private MotorController m_shooterRear = new WPI_VictorSPX(6);

  private CANSparkMax m_intake = new CANSparkMax(7, MotorType.kBrushed);

  private DifferentialDrive m_robotDrive = new DifferentialDrive(m_frontLeft::set, m_frontRight::set);

  private XboxController driver_controller = new XboxController(0);
  private XboxController shooter_controller = new XboxController(1);

  // motor constants
  private static double INPUT_SPEED = .50;
  private static double OUTPUT_SPEED_FRONT = 1;
  private static double OUTPUT_SPEED_REAR = 1;
  private static double INTAKE_SPEED = .25;

  // Current Limit Constatns
  static final boolean CURRTNE_LIMIT_ENABLED = true;
  static final int PEAK_CURRENT_AMPS = 40;
  static final int PEAK_CURRENT_DURATION_MS = 0;
  static final int CONTINUOUS_CURRENT_AMPS = 40;

  /* Current to maintain once current limit has been triggered */
  static final int kContinCurrentAmps = 10;
  
  // Create a time for the autonmous period
  private Timer timer = new Timer();

  //Autonomous Chooser This is used to select the auto mode
  private static final String DEFAULT = "Do Nothing";
  private static final String DRIVE = "Drive Forward and Stop";
  private static final String SHOOT = "Shoot and Drive Backward";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //Parameters for the DRIVE auto mode
  //These times represent when the next action starts
  static final double AUTO_DRIVE_SPEED = .5; //The speed to drive out for the auto drive mode
  static final double AUTO_DRIVE_START_TIME_S = 0; //When to start driving
  static final double AUTO_DRIVE_STOP_TIME_S = 2; //When to stop driving

  //These parameters control the shooting parameter
  //These times represent when the next action starts
  static final double AUTO_SHOOT_DRIVE_SPEED = .5;
  static final double AUTO_SHOOT_FRONT_TIME_S = 0; //When to start the front motor
  static final double AUTO_SHOOT_REAR_TIME_S = .5; //When to start the rear motor and leave the front motor on
  static final double AUTO_SHOOT_DRIVE_TIME_S = 1; //When stop the shooter and start driving forward
  static final double AUTO_SHOOT_STOP_TIME_S = 4; //When to stop driving

  public Robot() {
    SendableRegistry.addChild(m_robotDrive, m_frontLeft);
    SendableRegistry.addChild(m_robotDrive, m_rearLeft);
    SendableRegistry.addChild(m_robotDrive, m_frontRight);
    SendableRegistry.addChild(m_robotDrive, m_rearRight);
  }

  @Override
  public void robotInit() {

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rearLeft.set(TalonSRXControlMode.Follower,1);
    m_rearRight.set(TalonSRXControlMode.Follower,3);
    m_frontRight.setInverted(true);
    m_rearRight.setInverted(true);

    // Set the netural mode to brake. This will make the robot slowdown faster
    // and it will resist being pushed by other robots. 
    m_frontRight.setNeutralMode(NeutralMode.Brake);
    m_frontLeft.setNeutralMode(NeutralMode.Brake);
    m_rearRight.setNeutralMode(NeutralMode.Brake);
    m_rearLeft.setNeutralMode(NeutralMode.Brake);

    m_frontRight.configNeutralDeadband(0);
    m_frontLeft.configNeutralDeadband(0);
    m_rearRight.configNeutralDeadband(0);
    m_rearLeft.configNeutralDeadband(0);

    // Set the current limits
    m_frontLeft.enableCurrentLimit(CURRTNE_LIMIT_ENABLED);
    m_frontLeft.configContinuousCurrentLimit(CONTINUOUS_CURRENT_AMPS);
    m_frontLeft.configPeakCurrentLimit(PEAK_CURRENT_AMPS);
    m_frontLeft.configPeakCurrentDuration(PEAK_CURRENT_DURATION_MS);

    m_rearLeft.enableCurrentLimit(CURRTNE_LIMIT_ENABLED);
    m_rearLeft.configContinuousCurrentLimit(CONTINUOUS_CURRENT_AMPS);
    m_rearLeft.configPeakCurrentLimit(PEAK_CURRENT_AMPS);
    m_rearLeft.configPeakCurrentDuration(PEAK_CURRENT_DURATION_MS);

    m_frontRight.enableCurrentLimit(CURRTNE_LIMIT_ENABLED);
    m_frontRight.configContinuousCurrentLimit(CONTINUOUS_CURRENT_AMPS);
    m_frontRight.configPeakCurrentLimit(PEAK_CURRENT_AMPS);
    m_frontRight.configPeakCurrentDuration(PEAK_CURRENT_DURATION_MS);

    m_rearRight.enableCurrentLimit(CURRTNE_LIMIT_ENABLED);
    m_rearRight.configContinuousCurrentLimit(CONTINUOUS_CURRENT_AMPS);
    m_rearRight.configPeakCurrentLimit(PEAK_CURRENT_AMPS);
    m_rearRight.configPeakCurrentDuration(PEAK_CURRENT_DURATION_MS);

    //Autonomous Chooser
    m_chooser.setDefaultOption("Do Nothing", DEFAULT);
    m_chooser.addOption("Drive Forward and Stop", DRIVE);
    m_chooser.addOption("Shoot and Drive Backward", SHOOT);
    SmartDashboard.putData("Auto Choices", m_chooser);

    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
  }

  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    switch (m_autoSelected) {

      case DRIVE:
        if(timer.get() > AUTO_DRIVE_START_TIME_S && timer.get() < AUTO_DRIVE_STOP_TIME_S) {
          m_robotDrive.tankDrive(AUTO_DRIVE_SPEED, AUTO_DRIVE_SPEED);
        } else {
          m_robotDrive.tankDrive(0, 0);
        }
        break;

      case SHOOT:
        if(timer.get() > AUTO_SHOOT_FRONT_TIME_S && timer.get() < AUTO_SHOOT_REAR_TIME_S){
          m_shooterFront.set(OUTPUT_SPEED_FRONT);
        } else if( timer.get() > AUTO_SHOOT_REAR_TIME_S && timer.get() < AUTO_SHOOT_DRIVE_TIME_S) {
          m_shooterFront.set(OUTPUT_SPEED_FRONT);
          m_shooterRear.set(OUTPUT_SPEED_REAR);
        } else if ( timer.get() > AUTO_SHOOT_DRIVE_TIME_S && timer.get() < AUTO_SHOOT_STOP_TIME_S){
          m_shooterFront.set(0);
          m_shooterRear.set(0);
          m_robotDrive.tankDrive(AUTO_SHOOT_DRIVE_SPEED, AUTO_SHOOT_DRIVE_SPEED);
        } else {
          m_robotDrive.tankDrive(0,0);
        }
        break;

      default:
        m_robotDrive.tankDrive(0,0);
        m_shooterFront.set(0);
        m_shooterRear.set(0);
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    if(driver_controller.getRightBumper() && !shooter_controller.getYButton()){
      m_robotDrive.tankDrive(-driver_controller.getLeftY()*0.75, -driver_controller.getRightY()*0.75);
    } else if (!shooter_controller.getYButton()){
      m_robotDrive.tankDrive(-driver_controller.getLeftY(), -driver_controller.getRightY());
    } else {
      m_robotDrive.tankDrive(0, 0);
    }



    // Shooter Front
    if(shooter_controller.getYButton()){
      m_shooterFront.set(OUTPUT_SPEED_FRONT);
    } else if(shooter_controller.getAButton())
      m_shooterFront.set(-INPUT_SPEED);
    else {
      m_shooterFront.set(0);
    }

    // Shooter Rear
    if(shooter_controller.getRightBumper()){
      m_shooterRear.set(OUTPUT_SPEED_REAR);
    } else if(shooter_controller.getAButton())
      m_shooterRear.set(-INPUT_SPEED); 
    else {
      m_shooterRear.set(0);
    }
    
    //intake
    if (shooter_controller.getBButton()){
      m_intake.set(INTAKE_SPEED);
    } else if (shooter_controller.getXButton()) {
      m_intake.set(-INTAKE_SPEED);
    } else {
      m_intake.set(0);
    }
  }
}