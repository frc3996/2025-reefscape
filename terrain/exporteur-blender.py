import bpy
import json
import math
from pathlib import Path
from typing import List

def QuatVersDegree2D(q) -> float:
    q.normalize()
    return math.degrees(q.to_euler("XYZ")[2])

prefixesInteressants = tuple(["b_", "r_"])

objets = {} # liste de dictionnaires
for obj in bpy.data.objects:
    if obj.name.startswith(prefixesInteressants):
        position, rotation, scale = obj.matrix_world.decompose()
        objets[obj.name] = {'pos': [position.x, position.y], 'rot': QuatVersDegree2D(rotation)}

dossierMaison = Path.home()
with open(dossierMaison / 'terrain.json', 'w', encoding='utf-8') as fichier:
    json.dump(objets, fichier, ensure_ascii=False, indent=4)
