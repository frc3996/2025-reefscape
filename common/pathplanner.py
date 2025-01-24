from typing import TypedDict


class Position(TypedDict):
    x: float
    y: float


class StartingPose(TypedDict):
    position: Position
    rotation: float


class PathData(TypedDict):
    pathName: str
    name: str


class NamedData(TypedDict):
    name: str
    pathName: str  # Unused


class CommandPath(TypedDict):
    type: str  # "path"
    data: PathData


class CommandNamed(TypedDict):
    type: str  # "named"
    data: NamedData


# Union type for command types, since a command can either be "path" or "named"
CommandType = CommandPath | CommandNamed


class CommandData(TypedDict):
    name: str
    commands: list[CommandType]


class Command(TypedDict):
    type: str  # "sequential"
    data: CommandData


class PathPlannerAuto(TypedDict):
    version: float
    startingPose: StartingPose
    command: Command
    folder: str | None
    choreoAuto: bool
