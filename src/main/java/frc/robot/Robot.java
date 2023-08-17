// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private PWMSparkMax leftFront = new PWMSparkMax(0);
  private PWMSparkMax leftBack = new PWMSparkMax(1);
  private PWMSparkMax rightFront = new PWMSparkMax(2);
  private PWMSparkMax rightBack = new PWMSparkMax(3);

  private Encoder leftEncoder = new Encoder(0, 1);
  private Encoder rightEncoder = new Encoder(2, 3);

  private EncoderSim leftEncoderSim = new EncoderSim(leftEncoder);
  private EncoderSim rightEncoderSim = new EncoderSim(rightEncoder);

  private ADIS16470_IMU gyro = new ADIS16470_IMU();

  private ADIS16470_IMUSim gyroSim = new ADIS16470_IMUSim(gyro);

  private XboxController controller = new XboxController(0);
  private XboxControllerSim controllerSim = new XboxControllerSim(controller);

  private DifferentialDrivetrainSim drivetrainSim = DifferentialDrivetrainSim.createKitbotSim(
    KitbotMotor.kDoubleNEOPerSide, 
    KitbotGearing.k10p71,
    KitbotWheelSize.kSixInch, 
    null);

  private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
    Rotation2d.fromDegrees(gyro.getAngle()), 0, 0);

  private Field2d field = new Field2d();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code
   */
  @Override
  public void robotInit() {}

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    leftEncoder.setDistancePerPulse(Math.PI * Units.inchesToMeters(6) * 10 / 71d);
    rightEncoder.setDistancePerPulse(Math.PI * Units.inchesToMeters(6) * 10 / 71d);

    SmartDashboard.putData(field);
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    double throttle = -controller.getLeftY();
    double twist = controller.getRightX();

    if(Math.abs(throttle) < 0.1){
      throttle = 0;
    }
    if(Math.abs(twist) < 0.1){
      twist = 0;
    }

    leftFront.set(throttle + twist); 
    leftBack.set(throttle + twist); 
    rightBack.set(throttle - twist); 
    rightFront.set(throttle - twist);

    drivetrainSim.setInputs(
      leftFront.get() * RobotController.getInputVoltage(),
      rightFront.get() * RobotController.getInputVoltage()
    );

    drivetrainSim.update(0.02);

    leftEncoderSim.setDistance(drivetrainSim.getLeftPositionMeters());
    leftEncoderSim.setRate(drivetrainSim.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setDistance(drivetrainSim.getRightPositionMeters());
    rightEncoderSim.setRate(drivetrainSim.getRightVelocityMetersPerSecond());
    gyroSim.setGyroAngleZ(drivetrainSim.getHeading().getDegrees());
    controllerSim.setRawAxis(1, throttle);
    controllerSim.setRawAxis(4, twist);

    odometry.update(
      Rotation2d.fromDegrees(gyro.getAngle()), 
      leftEncoder.getDistance(),
      rightEncoder.getDistance());

    field.setRobotPose(odometry.getPoseMeters());
  }
}
