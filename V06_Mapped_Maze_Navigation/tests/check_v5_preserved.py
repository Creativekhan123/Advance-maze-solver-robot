"""Confirm V5's navigation decisions and maneuver counts remain unchanged."""
from pathlib import Path

project = Path(__file__).resolve().parents[1]
source = (project.parent / "V05_Left_Hand_Maze_Navigation" /
          "V05_Left_Hand_Maze_Navigation.ino").read_text()
expected = source[source.index("void autonomus() {"):source.index("void setup() {")]
expected = expected.replace("delay(rest);", "responsivePause(rest);").strip()
actual = (project / "main.cpp").read_text()
actual = actual[actual.index("void autonomus() {"):].strip()
assert actual == expected, "V5 navigation/maneuver body changed"
print("PASS: V5 navigation and maneuver calls preserved; waits are interruptible.")
