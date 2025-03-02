import json
from typing import List
from dataclasses import dataclass
from wpilib import DriverStation
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Transform2d
import wpimath.units
import os

NOM_FICHIER_TERRAIN : str = "terrain.json"
IS_FROM_EDIT_MODE_KEY : str = "isFromEditMode"
FIELD_LENGTH = 17.548
FIELD_WIDTH = 8.052

ROBOT_HALF_LENGTH = wpimath.units.inchesToMeters(17) # Incluant le bumper

def find_closest_pose(target: Pose2d, poses: list[Pose2d]) -> Pose2d:
    return min(poses, key=lambda p: p.translation().distance(target.translation()))

class Reefscape:

    def __init__(self):
        self.terrainJSON = {}
        pathFichierTerrain = os.path.join(os.path.dirname(__file__), r"..", NOM_FICHIER_TERRAIN)
        with open(pathFichierTerrain, 'r') as fichier:
            self.terrainJSON = json.load(fichier)
            self.poses = {}
            isFromEditMode = (IS_FROM_EDIT_MODE_KEY in self.terrainJSON) and self.terrainJSON[IS_FROM_EDIT_MODE_KEY]
            for cle in self.terrainJSON:
                if cle == IS_FROM_EDIT_MODE_KEY:
                    continue
                transform = self.terrainJSON[cle]
                locationJSON = transform["pos"]
                rotation = Rotation2d.fromDegrees(transform["rot"])
                offsetTranslation = Translation2d(0, 0)
                offsetRotation = Rotation2d.fromDegrees(0)
                if not isFromEditMode:
                    offsetTranslation = Translation2d(ROBOT_HALF_LENGTH * rotation.cos(), ROBOT_HALF_LENGTH * rotation.sin())
                    offsetRotation = Rotation2d.fromDegrees(180)
                location = Translation2d(float(locationJSON[0]) + offsetTranslation.x,
                                         float(locationJSON[1]) + offsetTranslation.y)
                self.poses[cle] = Pose2d(location, rotation.rotateBy(offsetRotation))
        if not self.ok():
            raise Exception("Terrain non chargé")

    def ok(self) -> bool:
        return len(self.poses) > 0
    
    def setCagePose(self, team : str, cage : int, pose : Pose2d):
        assert(cage in range(1, 4))
        self.poses[team + "_c" + str(cage)] = pose

    def setStationSlidePose(self, team : str, station : int, slide : int, pose : Pose2d):
        assert(station in range(1, 3))
        assert(slide in range(1, 5))
        self.poses[team + "_s" + str(station) + "_" + str(slide)] = pose

    def setReefPose(self, team : str, reef : int, pose : Pose2d):
        assert(reef in range(1, 13))
        self.poses[team + "_r" + str(reef)] = pose

    def save(self, filename : str):
        with open(filename, 'w') as fichier:
            self.terrainJSON = { }
            self.terrainJSON[IS_FROM_EDIT_MODE_KEY] = True 
            for pose in self.poses:
                self.terrainJSON[pose] = {
                    "pos": [self.poses[pose].translation().x, self.poses[pose].translation().y],
                    "rot": self.poses[pose].rotation().degrees()
                }
            json.dump(self.terrainJSON, fichier, indent=4)

    # Reef 1 à 12
    def getReef(self, numero : int) -> Pose2d:
        assert(numero in range(1, 13))
        return self.poses[self.__getPrefixeAllianceStr() + "_r" + str(numero)]

    # Station 1 (gauche) à 2 (droite)
    # Slide 1 à 4 (à partir du centre du terrain vers l'extérieur)
    def getCoralStationSlide(self, numeroStation : int, numeroSlide : int) -> Pose2d:
        assert(numeroStation in range(1, 3))
        assert(numeroSlide in range(1, 5))
        return self.poses[self.__getPrefixeAllianceStr() + "_s" + str(numeroStation) + "_" + str(numeroSlide)]

    def getClosestCoralStationSlide(self, numeroStation : int, target : Pose2d) -> Pose2d:
        return find_closest_pose(target, self.__getAllCoralStationSlides(numeroStation))

    # Cage 1 à 3
    def getCage(self, numero : int) -> Pose2d:
        assert(numero in range(1, 4))
        return self.poses[self.__getPrefixeAllianceStr() + "_c" + str(numero)]

    def getAllRedPoints(self) -> List[Pose2d]:
        return self.__getAllPointsWithPrefix("r_")

    def getAllBluePoints(self) -> List[Pose2d]:
        return self.__getAllPointsWithPrefix("b_")

    def __getAllCoralStationSlides(self, numeroStation : int) -> List[Pose2d]:
        return self.__getAllPointsWithPrefix(self.__getPrefixeAllianceStr() + "_s" + str(numeroStation) + "_")

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
