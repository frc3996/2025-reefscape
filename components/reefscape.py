import json
from typing import List
from dataclasses import dataclass
from wpilib import DriverStation
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Transform2d
import wpimath.units
import os

NOM_FICHIER_TERRAIN : str = "terrain.json"

FIELD_LENGTH = 17.548
FIELD_WIDTH = 8.052

ROBOT_HALF_LENGTH = wpimath.units.inchesToMeters(17) # INcluant le bumper

class Reefscape:

    def __init__(self):
        self.terrainJSON = None
        pathFichierTerrain = os.path.join(os.path.dirname(__file__), r"..", NOM_FICHIER_TERRAIN)
        with open(pathFichierTerrain, 'r') as fichier:
            self.terrainJSON = json.load(fichier)
            self.poses = {}
            for cle in self.terrainJSON:
                transform = self.terrainJSON[cle]
                locationJSON = transform["pos"]
                rotation = Rotation2d.fromDegrees(transform["rot"])
                offsetRobot = Translation2d(ROBOT_HALF_LENGTH * rotation.cos(), ROBOT_HALF_LENGTH * rotation.sin())
                location = Translation2d(float(locationJSON[0]) + offsetRobot.x,
                                         float(locationJSON[1]) + offsetRobot.y)
                self.poses[cle] = Pose2d(location, rotation.rotateBy(Rotation2d.fromDegrees(180)))
        if not self.ok():
            raise Exception("Terrain non chargé")

    def ok(self) -> bool:
        return len(self.poses) > 0

    # Reef 1 à 12
    def getReef(self, numero : int) -> Pose2d:
        assert(numero in range(1, 13))
        return self.poses[self.__getPrefixeAllianceStr() + "_r" + str(numero)]

    # Station 1 (gauche) à 2 (droite)
    # Slide 1 à 4
    def getCoralStationSlide(self, numeroStation : int, numeroSlide : int) -> Pose2d:
        assert(numeroStation in range(1, 3))
        assert(numeroSlide in range(1, 10))
        return self.poses[self.__getPrefixeAllianceStr() + "_s" + str(numeroStation) + "_" + str(numeroSlide)]

    def getAllCoralStationSlides(self, numeroStation : int) -> List[Pose2d]:
        return self.__getAllPointsWithPrefix(self.__getPrefixeAllianceStr() + "_s" + str(numeroStation) + "_")

    # Cage 1 à 3
    def getCage(self, numero : int) -> Pose2d:
        assert(numero in range(1, 4))
        return self.poses[self.__getPrefixeAllianceStr() + "_c" + str(numero)]

    def getAllRedPoints(self) -> List[Pose2d]:
        return self.__getAllPointsWithPrefix("r_")

    def getAllBluePoints(self) -> List[Pose2d]:
        return self.__getAllPointsWithPrefix("b_")

    def __getAllPointsWithPrefix(self, prefix : str) -> List[Pose2d]:
        points : List[Pose2d] = []
        for cle in self.poses:
            if cle.startswith(prefix):
                points.append(self.poses[cle])
        return points

    def __getPrefixeAllianceStr(self) -> str:
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            return "r"
        elif DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            return "b"
        else:
            raise Exception("Pick a side")

    def execute(self):
        pass
