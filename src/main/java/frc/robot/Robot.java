/**
 * Writen by Armando Mac Beath
 * 
 * {@MÆTH}
 */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Mecanismos.IntakeButtonCmd;
import frc.robot.commands.Mecanismos.PivoteoCommand;
import frc.robot.commands.Mecanismos.setPivotVelocity;
import frc.robot.commands.hybrid.subroutines;
import frc.robot.commands.hybrid.subroutines.ShooterState;
import frc.robot.subsystems.swerve.swerveSusbsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

   

   // CANSparkMax motorIzqPivoteo = new CANSparkMax(10, MotorType.kBrushless);

    //CANSparkMax motorDerPivoteo = new CANSparkMax(9, MotorType.kBrushless);

    Joystick placerJoystick = RobotContainer.placerJoystick;



    private RobotContainer m_robotContainer;
    swerveSusbsystem swerve;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        swerve = swerveSusbsystem.getInstance();
        swerve.resetEncoders();

        NamedCommands.registerCommand("shootFromSubwoofer", subroutines.shootWithDelay(subroutines.getShootingState(ShooterState.SHOOT_FROM_SUBWOOFER)));
        NamedCommands.registerCommand("lowArm", subroutines.lowArm());
        NamedCommands.registerCommand("lowArmAndShootFromSubwoofer", subroutines.lowArmAndShoot(subroutines.getShootingState(ShooterState.SHOOT_FROM_SUBWOOFER)));
        NamedCommands.registerCommand("intakeNote", new IntakeButtonCmd(-0.5, true));
        NamedCommands.registerCommand("aimAtSubwoofer", new PivoteoCommand(0.061).withTimeout(2));
        NamedCommands.registerCommand("lowPivotToGround", new setPivotVelocity(0));




        
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

    }


    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {

/* 

        if (placerJoystick.getRawButton(5)){
            //intakea
            Intake.setMotors(-0.4);
        } else if (placerJoystick.getRawButton(4)){
            //des-intakea
            Intake.setMotors(0.4);

        } else {
            Intake.setMotors(0);
            /* 
            if (placerJoystick.getPOV() == 90) {
                Shooter.setMotors(0.235);
                Intake.setMotors(0.35);
    
            } else {
                Shooter.setMotors(0);
                Intake.setMotors(0);
            }
            */

        
        }
        
        /* 
        if (placerJoystick.getPOV() == 90) {
            Shooter.setMotors(-0.2);
            Intake.setMotors(-0.35);

        } else if (placerJoystick.getRawButton(4)){
            Intake.setMotors(0.4);

        } else if (placerJoystick.getRawButton(5)){
            Intake.setMotors(-0.4);

        }else {
            Shooter.setMotors(0);
            Intake.setMotors(0);
        }
        */
        
        /* 
         
        if (placerJoystick.getRawButton(10)){ // joystick izq
            // dispara
            Shooter.setMotors(-0.7);
            

        }  else if (placerJoystick.getRawButton(9)){ // joystick der
            //escupir
            Shooter.setMotors(0.7);
        } else {
            Shooter.setMotors(0);
        }
            /* 
            if (placerJoystick.getPOV() == 90) {
                Shooter.setMotors(0.235);
                Intake.setMotors(0.35);
    
            } else {
                Shooter.setMotors(0);
                Intake.setMotors(0);
            }
             */
        

        
/* 
        if (placerJoystick.getRawButton(1)) {
            motorIzqPivoteo.set(0.17);
            motorDerPivoteo.set(-0.17);
        } else if (placerJoystick.getRawButton(2)){ // invertido CAMBIAR
            motorIzqPivoteo.set(0.05);
            motorDerPivoteo.set(-0.05);
        } else if (placerJoystick.getRawButton(9)){ 
            motorIzqPivoteo.set(-0.1);
            motorDerPivoteo.set(0.1);
        } else {
            motorIzqPivoteo.set(0);
            motorDerPivoteo.set(0);
        }

        */
    

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }
}
