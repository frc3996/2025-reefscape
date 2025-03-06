import os

import common.tools
import wpimath
import wpimath.geometry

from magicbot import feedback
from robotpy_apriltag import AprilTagFieldLayout
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from components.reefscape import Reefscape
from components.rikistick import RikiStick
from components.swervedrive import SwerveDrive
from common.graph import Graph, Node
from ntcore import NetworkTableInstance

# get the dir of THIS file (vision.py), go up one level (..), and use the specified filename
apriltagsFilename = r"2025-reefscape.json"
apriltagsLayoutPath = os.path.join(os.path.dirname(__file__), r"..", apriltagsFilename)

class FieldLayout(AprilTagFieldLayout):
    drivetrain: SwerveDrive
    rikiStick: RikiStick
    reefscape: Reefscape

    def __init__(self):
        super().__init__(apriltagsLayoutPath)
        self.graphRed : Graph = Graph()
        self.graphBlue : Graph = Graph()

    def setup(self) -> None:
        self.graphBlue = self.__setupGraphForTeam("b")
        self.graphRed = self.__setupGraphForTeam("r")
        #self.graphBlue.write("graphBlue.txt")
        #self.graphRed.write("graphRed.txt")
 
    def getGraph(self) -> Graph:
        if common.tools.is_red():
            return self.graphRed
        else:
            return self.graphBlue

    def __setupGraphForTeam(self, teamPrefix : str) -> Graph:
        points : list[tuple[str,Pose2d]] = []
        graph : Graph = Graph()
        if teamPrefix == "b":
            points = self.reefscape.getAllBlueNamedPoints()
        elif teamPrefix == "r":
            points = self.reefscape.getAllRedNamedPoints()
        else:
            raise Exception("No team")
 
        # Configurer le graph
        for point in points:
            graph.addNode(Node(point[0], point[1].translation()))

        # Ajouter les edges entre les pairs de reefs et leur point de transit
        for i in range(0, 6):
            iReef1 = 2*i + 1
            iReef2 = 2*i + 2
            graph.addEdgeBidirectional(teamPrefix + "_r" + str(iReef1), teamPrefix + "_t" + str(i + 1))
            graph.addEdgeBidirectional(teamPrefix + "_r" + str(iReef2), teamPrefix + "_t" + str(i + 1))
 
        # Ajouter les edges entre les points de transit
        for i in range(1, 7):
            graph.addEdgeBidirectional(teamPrefix + "_t" + str(i), teamPrefix + "_t" + str((i % 6) + 1))

        # Ajouter les edges entre les stations et certains points de transit
        transitPointsPourStations = { 1: ["t1", "t2", "t3"], 2: ["t1", "t5", "t6"] }
        for iStation in range(1,3): # [1,2]
            for transit in transitPointsPourStations[iStation]:
                for iSlide in range(1, 5): # [1,4]
                    graph.addEdgeBidirectional(teamPrefix + "_" + transit,
                                               teamPrefix + "_s" + str(iStation) + "_" + str(iSlide))

        # Ajouter les edges entre les cages et certains points de transit
        transitPointsPourCages = ["t2", "t3", "t4", "t5"]
        for iCage in range(1,4): # [1,3]
            for transit in transitPointsPourCages:
                    graph.addEdgeBidirectional(teamPrefix + "_" + transit,
                                               teamPrefix + "_c" + str(iCage))                    
        return graph

    @feedback
    def redTargets(self) -> list[Pose2d]:
        """Just to display things"""
        return self.reefscape.getAllRedPoints()
    
    @feedback
    def blueTargets(self) -> list[Pose2d]:
        """Just to display things"""
        return self.reefscape.getAllBluePoints()

    def getCoralTarget(self) -> str:
        return self.reefscape.getClosestCoralStationSlide(
            self.rikiStick.getCoralStationTarget(), self.drivetrain.getPose()
        )

    def getCoralTargetPosition(self) -> Pose2d:
        return self.reefscape.getPose(self.getCoralTarget())

    def getCageTarget(self) -> str:
        return self.reefscape.getCage(
            1
        )  # TODO la cage choisi par le gamepad avec la killswitch à off?
    
    def getCageTargetPosition(self) -> Pose2d:
        # TODO
        return self.reefscape.getPose(self.getCageTarget())

    def getReefTarget(self) -> str:
        return self.reefscape.getReef(self.rikiStick.getReefTarget())

    def getReefTargetPosition(self) -> Pose2d:
        return self.reefscape.getPose(self.getReefTarget())

    def getPose(self, poseName : str) -> Pose2d:
        return self.reefscape.getPose(poseName)

    def execute(self):
        pass
