"""WRS: one SOCP preferred direction, with plane/axis degeneracy handling."""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from wrs.assembly import DirectionConfig, solve_directions
from _direction_example import make_case, draw_result, print_result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', default='corner', choices=('plane','corner','channel','shaft','blind_shaft','blocked'))
    parser.add_argument('--headless', action='store_true')
    args = parser.parse_args()

    assembly, state, normals = make_case(args.case)
    config = DirectionConfig(method='socp', preferred_direction=(0,0,1))
    result = solve_directions(normals, config=config)
    print_result(args.case, result)
    if args.headless:
        return

    from wrs import wvw
    base = wvw.World(cam_pos=(.34,-.49,.31), cam_lookat_pos=(0,0,0), port=8892)
    base.set_caption(f'SOCP | {args.case} | blue optimal/preferred direction')
    draw_result(base, assembly, state, normals, result)
    base.run()


if __name__ == '__main__':
    main()
