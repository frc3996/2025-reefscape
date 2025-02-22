import bpy
import json
from pathlib import Path
from typing import List

# Conversion des types Blender vers liste de floats Python
def Vec2List(v) -> List[float]:
    return [v.x, v.y, v.z]

def Quat2List(q) -> List[float]:
    return [q.x, q.y, q.z, q.w]

prefixesInteressants = ["b_", "r_"]

objets = [] # liste de dictionnaires
for obj in bpy.data.objects:
    if obj.name.startswith(tuple(prefixesInteressants)):
        position, rotation, scale = obj.matrix_world.decompose()
        objets.append({'name': obj.name, 'pos': Vec2List(position), 'quat': Quat2List(rotation), 'scale': Vec2List(scale)})

dossierMaison = Path.home()
with open(dossierMaison / 'terrain.json', 'w', encoding='utf-8') as fichier:
    json.dump(objets, fichier, ensure_ascii=False, indent=4)
