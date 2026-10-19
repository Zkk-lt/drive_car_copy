"""Build isolated production-function tests; never start vehicle ROS nodes."""
from pathlib import Path
import shlex
import subprocess
import tempfile

root = Path(__file__).resolve().parents[3]
target = root / "build/move_car/CMakeFiles/control_pub.dir"
flags = dict(line.split(" = ", 1) for line in (target / "flags.make").read_text().splitlines()
             if " = " in line)
link = shlex.split((target / "link.txt").read_text())
with tempfile.TemporaryDirectory(prefix="control-regression-") as folder:
    for name in ("curve_control_regression", "follow_distance_regression", "collision_diagnostic_regression", "history_tracking_regression"):
        binary = str(Path(folder) / name)
        command = [link[0], "-std=c++14"] + shlex.split(flags["CXX_INCLUDES"])
        command += [str(Path(__file__).with_name(name + ".cpp")), "-o", binary]
        command += link[link.index("-o") + 2:]
        subprocess.run(command, cwd=root, check=True)
        subprocess.run([binary], cwd=root, check=True)
