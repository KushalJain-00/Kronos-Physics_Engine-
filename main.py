import argparse
from simulation.world import World
from visualization.renderer import Renderer
from scenes import SCENES


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Kronos physics engine")
    parser.add_argument("scene", nargs="?", default="sandbox", choices=sorted(SCENES.keys()),
                        help="Which demo scene to load (default: sandbox)")
    parser.add_argument("--width", type=int, default=800, help="World width")
    parser.add_argument("--height", type=int, default=600, help="World height")
    return parser


def main() -> None:
    args = build_argparser().parse_args()

    world = World(args.width, args.height)
    SCENES[args.scene](world)

    Renderer(world, scene=args.scene, scenes=SCENES).run()


if __name__ == "__main__":
    main()
