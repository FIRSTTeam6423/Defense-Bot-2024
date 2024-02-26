// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

import static edu.wpi.first.units.Units.Volts;

import javax.xml.xpath.XPathException;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.MutableMeasure.mutable;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Drive.SwerveModule;
import frc.robot.commons.IronUtil;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.SparkMaxLimitSwitch.Direction;

public class Drive extends SubsystemBase {
	private final Translation2d m_frontLeftLoc = new Translation2d(Constants.FRONTLEFT_X, Constants.FRONTLEFT_Y);
	private final Translation2d m_frontRightLoc = new Translation2d(Constants.FRONTRIGHT_X, Constants.FRONTRIGHT_Y);
	private final Translation2d m_backLeftLoc = new Translation2d(Constants.BACKLEFT_X, Constants.FRONTLEFT_Y);
	private final Translation2d m_backRightLoc = new Translation2d(Constants.BACKRIGHT_X, Constants.BACKRIGHT_Y);
	private Field2d f2d = new Field2d();
	private AHRS gyro = new AHRS();

	private final ModuleIO m_frontLeft;
	private final ModuleIO m_frontRight;
	private final ModuleIO m_backLeft;
	private final ModuleIO m_backRight;

	public Rotation2d simRotation = new Rotation2d();


    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data

	// WPILib
	StructArrayPublisher<SwerveModuleState> swervePublisher = NetworkTableInstance.getDefault()
		.getStructArrayTopic("SwerveStates", SwerveModuleState.struct).publish();

	public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(m_frontLeftLoc, m_frontRightLoc,
			m_backLeftLoc, m_backRightLoc);

	private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
	private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
	private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

	// getPosition is just placeholder for getting distance with encoders even
	// though wpilib uses it as an example
	// this took me like 30 min ot figure out
	// convert encoders to m

	private final SwerveDriveOdometry m_odometry;

	private final SysIdRoutine m_sysIdRoutine;

	public double deadzone(double input) {
		if (Math.abs(input) >= Constants.XBOX_STICK_DEADZONE_WIDTH) {
			return input;
		} else {
			return 0;
		}
	}

	public Rotation2d getHeading2d() {
		return gyro.getRotation2d();
	}

	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public ChassisSpeeds getRobotRelativeSpeeds() {
		return kinematics.toChassisSpeeds(getSwerveModuleStates()); // m_frontLeftLoc, m_frontRightLoc, m_backLeftLoc,
																	// m_backRightLoc
	}

	public Pose2d getAngleNegatedPose() {
		Pose2d p = getPose();
		return new Pose2d(p.getTranslation(), p.getRotation().times(-1));
	}

	public double getHeading() {
		return gyro.getYaw();
	}

	public double getPitch() {
		return gyro.getPitch();
	}

	public double getRoll() {
		return gyro.getRoll();
	}

	public SwerveModuleState[] getSwerveModuleStates() {
		// m_frontLeftLoc, m_frontRightLoc, m_backLeftLoc, m_backRightLoc
		SwerveModuleState[] states = { m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(),
				m_backRight.getState() };
		return states;
	}

	public void setSwerveModuleStates(SwerveModuleState[] states) {
		m_frontLeft.setDesiredState(states[0]);
		m_frontRight.setDesiredState(states[1]);
		m_backLeft.setDesiredState(states[2]);
		m_backRight.setDesiredState(states[3]);
	}

	public void resetPose(Pose2d pose) {
		m_odometry.resetPosition(Robot.isReal() ? getHeading2d() : simRotation, new SwerveModulePosition[] {
				m_frontLeft.getPosition(),
				m_frontRight.getPosition(),
				m_backLeft.getPosition(),
				m_backRight.getPosition()
		}, pose);
	}

	// -- COMANDS --

	public Command rotationTestCommand() {
		return this.run(() -> {
			ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 3);
			this.setChassisSpeeds(speeds);
		});
	}

	public Command driveRobot(boolean fieldRelative) {
		return this.runOnce(() -> {
			if(DriverStation.isAutonomous()) {
				setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
			} else {
				int xSign = (int) Math.signum(RobotContainer.getDriverLeftXboxY());
				double xSpeed = xSign * Math.pow(deadzone(RobotContainer.getDriverLeftXboxY()), 2)
						* Constants.MAX_LINEAR_SPEED
				// * Math.cos(Math.toRadians(RobotContainer.allianceOrientation))
						* ((RobotContainer.getDriverRightXboxTrigger() > .5) ? .25 : 1); // reversed x and y so that up on
																							// controller is

				int ySign = (int) Math.signum(RobotContainer.getDriverLeftXboxX());
				double ySpeed = ySign * Math.pow(deadzone(RobotContainer.getDriverLeftXboxX()), 2)
						* Constants.MAX_LINEAR_SPEED
				// * Math.cos(Math.toRadians(RobotContainer.allianceOrientation))
						* ((RobotContainer.getDriverRightXboxTrigger() > .5) ? .25 : 1); // reversed x and y so that up on
																							// controller is


				
				double omega = deadzone(IronUtil.powKeepSign(RobotContainer.getDriverRightXboxX(), 2.0))
						* Math.toRadians(Constants.MAX_ANGULAR_SPEED)
						* ((RobotContainer.getDriverRightXboxTrigger() > .5) ? .25 : 1);
					
				var speeds = ChassisSpeeds.fromFieldRelativeSpeeds(//ON CONTROLLER UP IS NEGATIVE
										-xSpeed, // reversed x and y so that up on controller is
										-ySpeed, // forward from driver pov
										-omega,
										getPose().getRotation());
				// SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
				// Constants.MAX_LINEAR_SPEED);
				
				setChassisSpeeds(speeds);
			}
		});
	}

	public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
		ChassisSpeeds speeds = new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, -chassisSpeeds.omegaRadiansPerSecond);
		setSwerveModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds));
	}

	public ChassisSpeeds getChassisSpeeds() {
		return kinematics.toChassisSpeeds(getSwerveModuleStates());
	}

	public Command flipOrientation() {
		return this.runOnce(() -> {
			Pose2d p = getPose();
			resetPose(new Pose2d(p.getTranslation(), p.getRotation().plus(Rotation2d.fromDegrees(180))));
		});
	}

	public Command driveForward() {
		return this.run(()->setChassisSpeeds(new ChassisSpeeds(1, 0, 0)));
	}

	public void configureAutos() {
		AutoBuilder.configureHolonomic(
				this::getPose, // Robot pose supplier
				this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
				this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
				new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
													// Constants class
						new PIDConstants(DriveConstants.AUTO_X_P, 0.0, 0.0), // Translation PID constants
						new PIDConstants(DriveConstants.AUTO_THETA_P, 0.0, 0.0), // Rotation PID constants
						4.5, // Max module speed, in m/s
						Units.inchesToMeters(16.6), // Drive base radius in meters. Distance from robot center to
													// furthest module. 16.6 inches
						new ReplanningConfig() // Default path replanning config. See the API for the options here
				),
				() -> {
					// Boolean supplier that controls when the path will be mirrored for the red
					// alliance
					// This will flip the path being followed to the red side of the field.
					// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

					var alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
						return alliance.get() == DriverStation.Alliance.Red;
					}
					return false;
				},
				this // Reference to this subsystem to set requirements
		);
	}

	/** Creates a new Drive. */
	public Drive() {
		if (Robot.isReal()) {
			m_frontLeft = new SwerveModule(
					Constants.FRONTLEFT_DRIVE,
					true,
					Constants.FRONTLEFT_PIVOT,
					Constants.FRONTLEFT_ABS_ENCODER, true);
			m_frontRight = new SwerveModule(
					Constants.FRONTRIGHT_DRIVE,
					true,
					Constants.FRONTRIGHT_PIVOT,
					Constants.FRONTRIGHT_ABS_ENCODER, true);
			m_backLeft = new SwerveModule(
					Constants.BACKLEFT_DRIVE,
					true,
					Constants.BACKLEFT_PIVOT,
					Constants.BACKLEFT_ABS_ENCODER, true);
			m_backRight = new SwerveModule(
					Constants.BACKRIGHT_DRIVE,
					true,
					Constants.BACKRIGHT_PIVOT,
					Constants.BACKRIGHT_ABS_ENCODER, true);
		} else {
			m_frontLeft = new SimModule(
					Constants.FRONTLEFT_DRIVE,
					true,
					Constants.FRONTLEFT_PIVOT,
					Constants.FRONTLEFT_ABS_ENCODER, true);
			m_frontRight = new SimModule(
					Constants.FRONTRIGHT_DRIVE,
					false,
					Constants.FRONTRIGHT_PIVOT,
					Constants.FRONTRIGHT_ABS_ENCODER, true);
			m_backLeft = new SimModule(
					Constants.BACKLEFT_DRIVE,
					false,
					Constants.BACKLEFT_PIVOT,
					Constants.BACKLEFT_ABS_ENCODER, true);
			m_backRight = new SimModule(
					Constants.BACKRIGHT_DRIVE,
					true,
					Constants.BACKRIGHT_PIVOT,
					Constants.BACKRIGHT_ABS_ENCODER, true);
		}
		gyro.reset();

		m_odometry = new SwerveDriveOdometry(kinematics, getHeading2d(),
				new SwerveModulePosition[] {
						m_frontLeft.getPosition(),
						m_frontRight.getPosition(),
						m_backLeft.getPosition(),
						m_backRight.getPosition()
				}, new Pose2d(0.0, 0.0, new Rotation2d()));
		m_sysIdRoutine = new SysIdRoutine(
				// Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
				new SysIdRoutine.Config(),
				new SysIdRoutine.Mechanism(
						// Tell SysId how to plumb the driving voltage to the motors.
						(Measure<Voltage> volts) -> {
							m_frontLeft.setVolts(volts.in(Volts), 0);
							m_frontRight.setVolts(volts.in(Volts), 0);
							m_backLeft.setVolts(volts.in(Volts), 0);
							m_backRight.setVolts(volts.in(Volts), 0);
						},
						// Tell SysId how to record a frame of data for each motor on the mechanism
						// being
						// characterized.
						log -> {
							log.motor("fl!!").voltage(
								m_appliedVoltage.mut_replace(
									m_frontLeft.getDriveVoltage(), Volts)
								).linearPosition(m_distance.mut_replace(
									m_frontLeft.getPosition().distanceMeters, Meters
								)).linearVelocity(m_velocity.mut_replace(
									m_frontLeft.getState().speedMetersPerSecond, MetersPerSecond
								));
							log.motor("fr!!").voltage(
								m_appliedVoltage.mut_replace(
									m_frontRight.getDriveVoltage(), Volts)
								).linearPosition(m_distance.mut_replace(
									m_frontRight.getPosition().distanceMeters, Meters
								)).linearVelocity(m_velocity.mut_replace(
									m_frontRight.getState().speedMetersPerSecond, MetersPerSecond
								));	
							log.motor("bl!!").voltage(
								m_appliedVoltage.mut_replace(
									m_backLeft.getDriveVoltage(), Volts)
								).linearPosition(m_distance.mut_replace(
									m_backLeft.getPosition().distanceMeters, Meters
								)).linearVelocity(m_velocity.mut_replace(
									m_backLeft.getState().speedMetersPerSecond, MetersPerSecond
								));	
							log.motor("br!!").voltage(
								m_appliedVoltage.mut_replace(
									m_backRight.getDriveVoltage(), Volts)
								).linearPosition(m_distance.mut_replace(
									m_backRight.getPosition().distanceMeters, Meters
								)).linearVelocity(m_velocity.mut_replace(
									m_backRight.getState().speedMetersPerSecond, MetersPerSecond
								));	
						},
						// Tell SysId to make generated commands require this subsystem, suffix test
						// state in
						// WPILog with this subsystem's name ("drive")
						this));

		// Configure pathplanner AutoBuilder
	}

	public Command runQuasistatic(SysIdRoutine.Direction dir) {
		return m_sysIdRoutine.quasistatic(dir);
	}

	public Command runDynamic(SysIdRoutine.Direction dir) {
		return m_sysIdRoutine.dynamic(dir);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		m_odometry.update(Robot.isReal() ? getHeading2d() : simRotation,
				new SwerveModulePosition[] {
						m_frontLeft.getPosition(), m_frontRight.getPosition(),
						m_backLeft.getPosition(), m_backRight.getPosition()
				});
		SmartDashboard.putNumber("rot deg", simRotation.getDegrees());
		SmartDashboard.putNumber("Rot speed", getChassisSpeeds().omegaRadiansPerSecond);
		f2d.setRobotPose(getPose());
		SmartDashboard.putData(f2d);
	}

	@Override
	public void simulationPeriodic() {
		simRotation = simRotation.rotateBy(
				Rotation2d.fromRadians(
						getChassisSpeeds().omegaRadiansPerSecond * 0.020));
		swervePublisher.set(getSwerveModuleStates());
	}
}
