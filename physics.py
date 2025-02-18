import typing

import rev
import wpilib
import wpilib.simulation
from pyfrc.physics.core import PhysicsInterface
from wpimath.geometry import Pose2d
from wpimath.system.plant import DCMotor

import constants

if typing.TYPE_CHECKING:
    from robot import MyRobot  # Basic Colors

# Basic Colors
RED = wpilib.Color8Bit(255, 0, 0)
GREEN = wpilib.Color8Bit(0, 255, 0)
BLUE = wpilib.Color8Bit(0, 0, 255)
WHITE = wpilib.Color8Bit(255, 255, 255)
BLACK = wpilib.Color8Bit(0, 0, 0)

# Bright Colors
YELLOW = wpilib.Color8Bit(255, 255, 0)
CYAN = wpilib.Color8Bit(0, 255, 255)
MAGENTA = wpilib.Color8Bit(255, 0, 255)
ORANGE = wpilib.Color8Bit(255, 165, 0)
PINK = wpilib.Color8Bit(255, 105, 180)

# Darker Colors
DARK_RED = wpilib.Color8Bit(139, 0, 0)
DARK_GREEN = wpilib.Color8Bit(0, 100, 0)
DARK_BLUE = wpilib.Color8Bit(0, 0, 139)
DARK_GRAY = wpilib.Color8Bit(64, 64, 64)

# Lighter Colors
LIGHT_RED = wpilib.Color8Bit(255, 102, 102)
LIGHT_GREEN = wpilib.Color8Bit(144, 238, 144)
LIGHT_BLUE = wpilib.Color8Bit(173, 216, 230)
LIGHT_GRAY = wpilib.Color8Bit(200, 200, 200)

# Fancy Colors
GOLD = wpilib.Color8Bit(255, 215, 0)
SILVER = wpilib.Color8Bit(192, 192, 192)
BRONZE = wpilib.Color8Bit(205, 127, 50)
LIME = wpilib.Color8Bit(50, 205, 50)
TEAL = wpilib.Color8Bit(0, 128, 128)
PURPLE = wpilib.Color8Bit(128, 0, 128)
BROWN = wpilib.Color8Bit(139, 69, 19)


class IntakeSimulator:
    def __init__(self, robot: "MyRobot"):
        self.robot: "MyRobot" = robot

        # Hardware SIM
        self.beamSensorSim: wpilib.simulation.AnalogInputSim = (
            wpilib.simulation.AnalogInputSim(self.robot.intake.beam_sensor)
        )
        self.intakeMotorSim = rev.SparkMaxSim(
            self.robot.intake.intake_motor, DCMotor.NEO()
        )
        self.outputMotorSim = rev.SparkMaxSim(
            self.robot.intake.output_motor, DCMotor.NEO()
        )

        # Create Mechanism2d displays
        self.mech2d = wpilib.Mechanism2d(6, 3)
        self.root = self.mech2d.getRoot("Root", 1, 1)

        # Intake motor
        o1_root = self.mech2d.getRoot("o1_root", 1, 1)
        o1_left = o1_root.appendLigament("o1_top", 1, 90, 5)
        o1_top = o1_left.appendLigament("o1_right", 1, -90, 5)
        o1_right = o1_top.appendLigament("o1_bottom", 1, -90, 5)
        o1_bottom = o1_right.appendLigament("o1_left", 1, -90, 5)
        self.o1 = [o1_root, o1_left, o1_top, o1_right, o1_bottom]

        # Beam
        l1_root = self.mech2d.getRoot("l1_root", 3, 1)
        l1_left = l1_root.appendLigament("l1_left", 1, 90, 5)
        self.l = [l1_root, l1_left]

        # Outout motor
        o2_root = self.mech2d.getRoot("o2_root", 4, 1)
        o2_left = o2_root.appendLigament("o2_top", 1, 90, 5)
        o2_top = o2_left.appendLigament("o2_right", 1, -90, 5)
        o2_right = o2_top.appendLigament("o2_bottom", 1, -90, 5)
        o2_bottom = o2_right.appendLigament("o2_left", 1, -90, 5)
        self.o2 = [o2_root, o2_left, o2_top, o2_right, o2_bottom]

        # Timer
        self._intake_timer: float = 0
        self._output_timer: float = 0
        self.beamSensorSim.setVoltage(4)

        # Put Mechanism to SmartDashboard
        wpilib.SmartDashboard.putData("Intake System Sim", self.mech2d)

    def simulationPeriodic(self):
        # Update motor visual representation based on speed
        intake_speed = self.robot.intake.intake_motor.get()
        output_speed = self.robot.intake.output_motor.get()

        applyColor(
            wpilib.Color8Bit(
                abs(int(intake_speed * 1024)) if intake_speed < 0 else 0,
                abs(int(intake_speed * 1024)) if intake_speed > 0 else 0,
                255 if intake_speed == 0 else 0,
            ),
            self.o1,
        )
        # Some guilty knowledge of the statemachine
        if intake_speed > 0 and output_speed < 0:
            self._intake_timer += 0.02
        else:
            self._intake_timer = 0
        if self._intake_timer > 2:
            self.beamSensorSim.setVoltage(0.1)

        # Output
        applyColor(
            wpilib.Color8Bit(
                abs(int(output_speed * 1024)) if output_speed < 0 else 0,
                abs(int(output_speed * 1024)) if output_speed > 0 else 0,
                255 if intake_speed == 0 else 0,
            ),
            self.o2,
        )
        if intake_speed > 0 and output_speed > 0:
            self._output_timer += 0.02
        else:
            self._output_timer = 0
        if self._output_timer > 0.4:
            self.beamSensorSim.setVoltage(4)

        # Change beam sensor color if triggered
        if self.robot.intake.piece_chargee():
            applyColor(wpilib.Color8Bit(255, 0, 0), self.l)
        else:
            applyColor(wpilib.Color8Bit(0, 255, 0), self.l)


class LiftSimulator:
    def __init__(self, robot: "MyRobot"):
        self.robot: "MyRobot" = robot

        # Sim hardware
        self.limitswitchZeroSim = wpilib.simulation.DIOSim(
            self.robot.lift.limitswitchZero
        )
        self.limitswitchSafetySwitch = wpilib.simulation.DIOSim(
            self.robot.lift.limitswitchSafety
        )
        self.stringEncoderSim: wpilib.simulation.EncoderSim = (
            wpilib.simulation.EncoderSim(self.robot.lift.stringEncoder)
        )

        # Initial state for limitswitches
        self.limitswitchZeroSim.setValue(False)
        self.limitswitchSafetySwitch.setValue(False)

        # Create a Mechanism2d display of an elevator
        self.mech2d = wpilib.Mechanism2d(1, 3.0)
        self.elevatorRoot = self.mech2d.getRoot("Elevator Root", 0.5, 0)
        self.elevatorMech2d = self.elevatorRoot.appendLigament(
            "Elevator",
            self.robot.lift.get_distance(),
            90,
            30,
            wpilib.Color8Bit(255, 0, 0),
        )

        # Box on top of the Elevator
        self.elevatorBox = self.elevatorMech2d.appendLigament(
            "Box",
            0.2,
            0,
            60,
            wpilib.Color8Bit(255, 0, 0),  # Small horizontal ligament as a box
        )

        # Put Mechanism to SmartDashboard
        wpilib.SmartDashboard.putData("Elevator Sim", self.mech2d)

    def simulationPeriodic(self):
        rate = self.robot.lift.liftMaster.get()
        self.stringEncoderSim.setRate(
            self.stringEncoderSim.getRate() + rate * self.robot.lift.kMaxSpeed
        )
        self.stringEncoderSim.setDistance(
            self.robot.lift.stringEncoder.getDistance() + rate * 0.02
        )

        self.elevatorMech2d.setLength(self.robot.lift.get_distance())


def applyColor(
    color: wpilib.Color8Bit,
    ligaments: list[wpilib.MechanismLigament2d | wpilib.MechanismRoot2d],
):
    for l in ligaments:
        if isinstance(l, wpilib.MechanismLigament2d):
            l.setColor(color)


class ClimbSimulator:
    def __init__(self, robot: "MyRobot"):
        self.robot: "MyRobot" = robot

        # Sim hardware
        self.limitswitchPiston1Sim = wpilib.simulation.DIOSim(
            self.robot.climb.piston_out_limitswitch_1
        )
        self.limitswitchPiston2Sim = wpilib.simulation.DIOSim(
            self.robot.climb.piston_out_limitswitch_2
        )
        self.limitswitchCage1Sim = wpilib.simulation.DIOSim(
            self.robot.climb.cage_in_limitswitch_1
        )
        self.limitswitchCage2Sim = wpilib.simulation.DIOSim(
            self.robot.climb.cage_in_limitswitch_2
        )

        self.climbEncoderSim = rev.SparkRelativeEncoderSim(self.robot.climb.climbMain)
        self.guideEncoderSim = rev.SparkRelativeEncoderSim(self.robot.climb.guideMotor)

        # Initial states for limitswitch
        self.limitswitchPiston1Sim.setValue(False)
        self.limitswitchPiston2Sim.setValue(False)
        self.limitswitchCage1Sim.setValue(False)
        self.limitswitchCage2Sim.setValue(False)

        # Create a Mechanism2d display of an elevator
        self.mech2d = wpilib.Mechanism2d(50.0, 50.0)

        # Root position for "p"
        p_root = self.mech2d.getRoot("p_root", 10, 10)
        p_left = p_root.appendLigament("p_main", 10, 90, 5)
        p_top = p_left.appendLigament("p_curve1", 5, -90, 5)
        p_right = p_top.appendLigament("p_curve2", 5, -90, 5)
        p_bottom = p_right.appendLigament("p_curve3", 5, -90, 5)
        self.p = [p_root, p_left, p_top, p_right, p_bottom]

        # Root position for first "o"
        o1_root = self.mech2d.getRoot("o1_root", 18, 20)
        o1_left = o1_root.appendLigament("o1_top", 5, 90, 5)
        o1_top = o1_left.appendLigament("o1_right", 5, -90, 5)
        o1_right = o1_top.appendLigament("o1_bottom", 5, -90, 5)
        o1_bottom = o1_right.appendLigament("o1_left", 5, -90, 5)
        self.o1 = [o1_root, o1_left, o1_top, o1_right, o1_bottom]

        # Root position for second "o"
        o2_root = self.mech2d.getRoot("o2_root", 32, 20)
        o2_left = o2_root.appendLigament("o2_top", 5, 90, 5)
        o2_top = o2_left.appendLigament("o2_right", 5, -90, 5)
        o2_right = o2_top.appendLigament("o2_bottom", 5, -90, 5)
        o2_bottom = o2_right.appendLigament("o2_left", 5, -90, 5)
        self.o2 = [o2_root, o2_left, o2_top, o2_right, o2_bottom]

        # First L
        l1_root = self.mech2d.getRoot("l1_root", 28, 25)
        l1_left = l1_root.appendLigament("l1_left", 5, -90, 50)
        self.l = [l1_root, l1_left]

        # Root position for first "o"
        o3_root = self.mech2d.getRoot("o3_root", 18, 5)
        o3_left = o3_root.appendLigament("o3_top", 5, 90, 5)
        o3_top = o3_left.appendLigament("o3_right", 5, -90, 5)
        o3_right = o3_top.appendLigament("o3_bottom", 5, -90, 5)
        o3_bottom = o3_right.appendLigament("o3_left", 5, -90, 5)
        self.o3 = [o3_root, o3_left, o3_top, o3_right, o3_bottom]

        # Root position for second "o"
        o4_root = self.mech2d.getRoot("o4_root", 32, 5)
        o4_left = o4_root.appendLigament("o4_top", 5, 90, 5)
        o4_top = o4_left.appendLigament("o4_right", 5, -90, 5)
        o4_right = o4_top.appendLigament("o4_bottom", 5, -90, 5)
        o4_bottom = o4_right.appendLigament("o4_left", 5, -90, 5)
        self.o4 = [o4_root, o4_left, o4_top, o4_right, o4_bottom]

        # Root position for "q"
        q_root = self.mech2d.getRoot("q_root", 45, 10)
        q_top = q_root.appendLigament("q_top", 10, 90, 5)
        q_right = q_top.appendLigament("q_right", 5, 90, 5)
        q_bottom = q_right.appendLigament("q_bottom", 5, 90, 5)
        q_left = q_bottom.appendLigament("q_left", 5, 90, 5)
        self.q = [q_root, q_top, q_right, q_bottom, q_left]

        self._angle: float = 0

        # Put Mechanism to SmartDashboard
        wpilib.SmartDashboard.putData("Climb Sim", self.mech2d)

    def simulationPeriodic(self):
        #### Guide motor
        # green is clockwise, red is counterclockwise, blue is idle
        rate = self.robot.climb.guideMotor.get()
        _ = self.guideEncoderSim.setVelocity(
            self.guideEncoderSim.getVelocity() * 0.95
            + rate * self.robot.climb.kGuideMaxSpeed
        )
        _ = self.guideEncoderSim.setPosition(
            self.guideEncoderSim.getPosition()
            + self.guideEncoderSim.getVelocity() * 0.02
        )
        if rate == 0:
            applyColor(BLUE, self.o1)
            applyColor(BLUE, self.o2)
        elif rate < 0:
            applyColor(RED, self.o1)
            applyColor(RED, self.o2)
        elif rate > 0:
            applyColor(GREEN, self.o1)
            applyColor(GREEN, self.o2)

        if self.guideEncoderSim.getVelocity() > self.robot.climb._guideSpeed:
            self.limitswitchCage1Sim.setValue(True)
            self.limitswitchCage2Sim.setValue(True)
            assert self.robot.climb.isCageIn()
        elif self.guideEncoderSim.getVelocity() < 0.01:
            self.limitswitchCage1Sim.setValue(False)
            self.limitswitchCage2Sim.setValue(False)
            assert not self.robot.climb.isCageIn()

        # Piston
        if self.robot.climb._solenoid.get():
            if self._angle < 45:
                self._angle += 1
            else:
                self.limitswitchPiston1Sim.setValue(True)
                self.limitswitchPiston2Sim.setValue(True)
        else:
            if self._angle > 0:
                self._angle -= 1
            else:
                self.limitswitchPiston1Sim.setValue(False)
                self.limitswitchPiston2Sim.setValue(False)
        self.p[1].setAngle(90 - self._angle)
        self.q[1].setAngle(90 + self._angle)

        # Grab the cage
        engaged = self.robot.climb.isCageSqueezed()
        applyColor(RED if engaged > 0 else GREEN, self.p)
        applyColor(RED if engaged > 0 else GREEN, self.q)

        #### Climb motor
        rate = self.robot.climb.climbMain.get()
        _ = self.climbEncoderSim.setVelocity(
            self.climbEncoderSim.getVelocity() * 0.95
            + rate * self.robot.climb.kClimbMaxSpeed
        )
        _ = self.climbEncoderSim.setPosition(
            self.climbEncoderSim.getPosition()
            + self.climbEncoderSim.getVelocity() * 0.02
        )
        if rate == 0:
            applyColor(BLUE, self.o3)
            applyColor(BLUE, self.o4)
        elif rate < 0:
            applyColor(RED, self.o3)
            applyColor(RED, self.o4)
        elif rate > 0:
            applyColor(GREEN, self.o3)
            applyColor(GREEN, self.o4)

        self.l[1].setLength(5 + self.climbEncoderSim.getPosition())


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot object
        """
        self.robot = robot

        self.navx = wpilib.simulation.SimDeviceSim("navX-Sensor[2]")
        self.navx_yaw = self.navx.getDouble("Yaw")
        self.navx_rate = self.navx.getDouble("Rate")

        self.liftSimulator: LiftSimulator = LiftSimulator(robot)
        self.climbSimulator: ClimbSimulator = ClimbSimulator(robot)
        self.intakeSimulator: IntakeSimulator = IntakeSimulator(robot)

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
        # SimBattery estimates loaded battery voltage
        wpilib.simulation.RoboRioSim.setVInVoltage(
            wpilib.simulation.BatterySim.calculate([0])
        )

        # Call the components periodic
        self.robot.drivetrain.simulationPeriodic()
        self.liftSimulator.simulationPeriodic()
        self.climbSimulator.simulationPeriodic()
        self.intakeSimulator.simulationPeriodic()

        # NavX
        self.navx_yaw.set(-self.robot.drivetrain.getPose().rotation().degrees())
        # self.navx_yaw.set(
        #     self.navx_yaw.get()
        #     + -1.0 * self.robot.drivetrain.getChassisSpeeds().omega_dps * 0.02
        # )
        # self.navx_rate.set(-1.0 * self.robot.drivetrain.getChassisSpeeds().omega_dps)
        # self.navx_yaw.set(self.navx_yaw.get() + self.navx_rate.get() * 0.02)

        self.navx_rate.set(-1.0 * self.robot.drivetrain.getChassisSpeeds().omega_dps)
        self.navx_yaw.set(self.navx_yaw.get() + self.navx_rate.get() * 0.02)
