// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  private WPI_TalonSRX leftMaster = new WPI_TalonSRX(3);
  private WPI_TalonSRX rightMaster = new WPI_TalonSRX(1);
  private WPI_VictorSPX leftSlave = new WPI_VictorSPX(1);
  private WPI_VictorSPX rightSlave = new WPI_VictorSPX(2);

  private WPI_TalonSRX armMotor = new WPI_TalonSRX(5);
  private WPI_VictorSPX armSlave = new WPI_VictorSPX(3);

  private WPI_TalonSRX rollerMotor = new WPI_TalonSRX(4);

  private Compressor compressor = new Compressor(null);
  private DoubleSolenoid hatchIntake = new DoubleSolenoid(null, 0, 1); // PCM Port 0, 1

  private DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);


  // joysticks
  private Joystick driverJoystick = new Joystick(0); // for the driver to drive the robot
  private Joystick  operatorJoystick = new Joystick(1);  // for an operator to control the mechanisms

  // unit conversion
  private final double kDriveTick2Feet = 1.0 / 4096 * 6 * Math.PI / 12;
  private final double kArmTick2Deg = 360.0 / 512 * 26 / 42 * 18 / 60 * 18 / 84;


  @Override
  public void robotInit() {
    // will run once when the robot is powered on
    leftMaster.setInverted(true);
    rightMaster.setInverted(true);
    armMotor.setInverted(false);

    // slave setups
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    armSlave.follow(armMotor);

    leftSlave.setInverted(InvertType.FollowMaster);
    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);
    armSlave.setInverted(InvertType.FollowMaster);

    // setting up encoders
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10); 
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10); 
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10); 
     
    leftMaster.setSensorPhase(false);
    rightMaster.setSensorPhase(true);
    armMotor.setSensorPhase(true);

    // reset encoder values to zero
    leftMaster.setSelectedSensorPosition(0, 0, 10);
    rightMaster.setSelectedSensorPosition(0, 0, 10);
    armMotor.setSelectedSensorPosition(0, 0, 10);
    
    // set encoder boundary limits: to stop motors
    armMotor.configReverseSoftLimitThreshold((int)(0 / kArmTick2Deg), 10);
    armMotor.configForwardSoftLimitThreshold((int)(175 / kArmTick2Deg), 10);

    armMotor.configReverseSoftLimitEnable(true, 10);
    armMotor.configForwardSoftLimitEnable(true, 10);


    // start compressor
    compressor.start();

    drive.setDeadband(0.05);
     
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Arm Encoder Value", armMotor.getSelectedSensorPosition() * kArmTick2Deg);
    SmartDashboard.putNumber("Left Drive Encoder Value", leftMaster.getSelectedSensorPosition() * kDriveTick2Feet);
    SmartDashboard.putNumber("Right Drive Encoder Value", rightMaster.getSelectedSensorPosition() * kDriveTick2Feet);

  }

  @Override
  public void autonomousInit() {
    enableMotors(true);
    // reset encoders to zero
    leftMaster.setSelectedSensorPosition(0, 0, 10);
    rightMaster.setSelectedSensorPosition(0, 0, 10);
    armMotor.setSelectedSensorPosition(0, 0, 10);
  }
  @Override
  public void autonomousPeriodic() {
    double leftPosition = leftMaster.getSelectedSensorPosition() * kDriveTick2Feet;
    double rightPosition = rightMaster.getSelectedSensorPosition() * kDriveTick2Feet;
    double distance = (leftPosition + rightPosition) / 2;
     
    if (distance < 10)
    {
      drive.tankDrive(0.6, 0.6);
    } 
    else
    {
      drive.tankDrive(0, 0);
    }
  }

  @Override
  public void teleopInit() 
  {
    enableMotors(true);
  }

  @Override
  public void teleopPeriodic() {
    double power = -driverJoystick.getRawAxis(1);
    double turn = driverJoystick.getRawAxis(4);

    drive.arcadeDrive(power * 0.6, turn * 0.3);

    // arm control
    double armPower = -operatorJoystick.getRawAxis(1);
    if (Math.abs(armPower)<0.05)
    {
      armPower = 0;
    }
    armPower *= 0.5;
    armMotor.set(ControlMode.PercentOutput, armPower);

    // roller control
    double rollerPower = 0;
    if (operatorJoystick.getRawButton(1) == true)
    {
      rollerPower = 1;
    }
    else if (operatorJoystick.getRawButton(2))
    {
      rollerPower = -1;
    }
    rollerMotor.set(ControlMode.PercentOutput, rollerPower);

    // hatch intake
    if (operatorJoystick.getRawButton(3))
    {
      hatchIntake.set(Value.kReverse);
    }
    else
    {
      hatchIntake.set(Value.kForward);
    }
  }


  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void disabledInit() {
    enableMotors(false);
  }

  // brake mode (enabled) - lets motors resist outside forces using back EMF, has the same effect of having more friction
// coast mode (disabled) - lets motors to be freely rotated by outside forces

  private void enableMotors(boolean on)
  {
    NeutralMode mode;
    if(on)
    {
      mode = NeutralMode.Brake;
    }
    else
    {
      mode = NeutralMode.Coast;
    }
      leftMaster.setNeutralMode(mode);
      rightMaster.setNeutralMode(mode);
      leftSlave.setNeutralMode(mode);
      rightSlave.setNeutralMode(mode);
      armMotor.setNeutralMode(mode);
      armSlave.setNeutralMode(mode); 
      rollerMotor.setNeutralMode(mode); 
  }
}
