import argparse
from simulation.world import World
from visualization.renderer import Renderer
from ui.control_panel import ControlPanel
from scenes import SCENES


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Kronos physics engine")
    parser.add_argument("scene", nargs="?", default="pendulum", choices=sorted(SCENES.keys()), help="Which demo scene to load (default: pendulum)")
    parser.add_argument("--width", type=int, default=800, help="Window width")
    parser.add_argument("--height", type=int, default=600, help="Window height")
    parser.add_argument("--no-panel", action="store_true", help="Run without the control panel")
    return parser

def main() -> None:
    args = build_argparser().parse_args()

    world = World(args.width, args.height)
    SCENES[args.scene](world)  # each scene function populates the world

    renderer = Renderer(world)

    if not args.no_panel:
        panel = ControlPanel(world)
        panel.start()

    renderer.run()


if __name__ == "__main__":
    main()