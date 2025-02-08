from terrain import Terrain

class Pathfinder:
    def __init__(self, terrain : Terrain):
        self.terrain = terrain

# terrain = Terrain('terrain/terrain.json')
# for point in terrain.obtenirPoints("d_"):
#     print(f"Name: {point.name}, Pos: {point.pos}, Quat: {point.quat}, Scale: {point.scale}")
