"""Run production history-merge regression without starting ROS nodes."""
from pathlib import Path
import shlex
import subprocess
import tempfile


root = Path(__file__).resolve().parents[3]
target = root / "build/move_car/CMakeFiles/trajectory_reconstructor.dir"
flags = dict(
    line.split(" = ", 1)
    for line in (target / "flags.make").read_text().splitlines()
    if " = " in line
)
link = shlex.split((target / "link.txt").read_text())
with tempfile.TemporaryDirectory(prefix="history-tail-test-") as folder:
    binary = str(Path(folder) / "regression")
    command = [link[0], "-std=c++14", "-fno-access-control"]
    command += shlex.split(flags["CXX_INCLUDES"])
    command += [str(Path(__file__).with_suffix(".cpp").with_name("history_tail_regression.cpp")),
                "-o", binary]
    command += link[link.index("-o") + 2:]
    subprocess.run(command, cwd=root, check=True)
    subprocess.run([binary], cwd=root, check=True)
