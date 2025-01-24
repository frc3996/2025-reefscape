# Base Swerve

Code de base pour un robot basé sur 4 swerves.

Pour rendre le robot fonctionnel, vous devez:
- Connecter un NAVX sur le RoboRIO
- Assigner le CAN id sur chaque contrôleur de moteur (2 par swerve modules). Bien noter quel moteur est où
- Assigner le CAN id sur les encodeurs (1 par swerve module)
- Ajuster le code de `robot.py` à la fonction `initSwerve()` avec ces id
- Mettre votre robot sur le côté
- Téléverser le code avec la commande `python3 robot.py deploy --nc` (ou `py -3 robot.py deploy --nc`)
- Ouvrir le ShuffleBoard via de Driver Station
- Faire bouger les roues et s'assurer qu'elles pointent dans la même direction en ajustant les valeurs:
    - frontLeftModule/rotation_zero
    - frontRightModule/rotation_zero
    - rearLeftModule/rotation_zero
    - rearRightModule/rotation_zero

Une fois que vous avez trouvé les bonnes valeurs, écrivez les dans `initSwerve()` comme valeur par défaut

```python
# Le ShuffleBoard est utilisé afin d'ajuster le zéro des roues.
# Un fois testé, les valeurs peuvent-être modifiées ici.
self.nt.putNumber("frontLeftModule/rotation_zero", 166)
self.nt.putNumber("frontRightModule/rotation_zero", 109)
self.nt.putNumber("rearLeftModule/rotation_zero", 330)
self.nt.putNumber("rearRightModule/rotation_zero", 44)
```
