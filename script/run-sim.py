from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage

# Replace 'path_to_your_scene.usd' with the actual file path
usd_path = "isaac-sim/ros2-turtlebot.usd"
open_stage(usd_path=usd_path)

world = World()
world.reset()

while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
