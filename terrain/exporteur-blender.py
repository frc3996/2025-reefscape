import bpy
import json
from pathlib import Path

def Vec2Str(v) -> str:
    return str(v.x) + ", " + str(v.y) + ", " + str(v.z)

def Quat2Str(q) -> str:
    return str(q.x) + ", " + str(q.y) + ", " + str(q.z) + ", " + str(q.w)

prefixesInteressants = ["g_", "d_"]

objets = [] # liste de dictionnaires
for obj in bpy.data.objects:
    if obj.name.startswith(tuple(prefixesInteressants)):
        location, rotation, scale = obj.matrix_world.decompose()
        objets.append({'name': obj.name, 'pos': Vec2Str(location), 'quat': Quat2Str(rotation), 'scale': Vec2Str(scale)})

dossierMaison = Path.home()
with open(dossierMaison / 'terrain.json', 'w', encoding='utf-8') as fichier:
    json.dump(objets, fichier, ensure_ascii=False, indent=4)
