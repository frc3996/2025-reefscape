import json
from typing import List
from dataclasses import dataclass

@dataclass
class TerrainPoint:
    def __init__(self, name : str, pos : list[float], quat : List[float], scale : List[float]):
        assert(len(pos) == 3)
        assert(len(quat) == 4)
        assert(len(scale) == 3)
        self.name = name
        self.pos = pos
        self.quat = quat
        self.scale = scale

class Terrain:
    def __init__(self, nomFichierJson : str):
        self.points = list[TerrainPoint]()
        self.nomFichierJson = "" # set dans charger
        self.charger(nomFichierJson)

    def charger(self, nomFichierJson : str):
        assert(len(nomFichierJson) > 0)
        self.nomFichierJson = nomFichierJson
        with open(nomFichierJson, 'r') as fichier:
            terrainJSON = json.load(fichier)
        self.points = list[TerrainPoint]()
        for point in terrainJSON:
            self.points.append(TerrainPoint(
                point['name'], 
                [float(x) for x in point['pos']], 
                [float(x) for x in point['quat']], 
                [float(x) for x in point['scale']]
            ))

    def obtenirPoints(self, prefixPoint : str = "") -> List[TerrainPoint]:
        return [p for p in self.points if p.name.startswith(prefixPoint)]
