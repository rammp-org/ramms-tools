#!/usr/bin/env python3
"""
ramms-worldgen — drive RammsWorldGen (procedural pedestrian / wheelchair worlds)
in a running editor or game over the Remote Control API.

    ramms-worldgen world                              # which map is open, how many generators
    ramms-worldgen presets                            # list presets (name, kind, site size)
    ramms-worldgen maps                               # list the generated example maps
    ramms-worldgen open Map_Gen_Suburban              # load / travel to a map
    ramms-worldgen spawn --preset PRE_Downtown --seed 7
    ramms-worldgen generate --seed 42                 # regenerate the (first) generator, wait, print summary
    ramms-worldgen randomize
    ramms-worldgen status
    ramms-worldgen params                             # resolved parameter block
    ramms-worldgen params --set HazardDensity=6 --set NonCompliance=0.5
    ramms-worldgen params --clear
    ramms-worldgen manifest --out downtown_42.json    # fetch the manifest
    ramms-worldgen manifest --issues                  # list non-compliant features
    ramms-worldgen manifest --routes                  # list routes with clear widths
    ramms-worldgen batch --preset PRE_Park --seeds 1-50 --out data/park
    ramms-worldgen inspect data/park/PRE_Park_3.json  # summarize a saved manifest (no engine needed)

Requires Unreal Engine running with the Remote Control API plugin (port 30010)
and the RammsWorldGen plugin.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path

from ramms_tools.unreal_remote import UnrealRemoteError
from ramms_tools.worldgen import WorldGen, WorldGenError
from ramms_tools.worldgen.manifest import Manifest


def _seeds(spec: str) -> list[int]:
    """'1-20' | '1,2,5' | '7' | '1-5,9'"""
    out: list[int] = []
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part[1:]:
            a, b = part.split("-", 1)
            out.extend(range(int(a), int(b) + 1))
        else:
            out.append(int(part))
    return out


def _print_summary(m: Manifest) -> None:
    s = m.summary()
    print(f"{s['preset']}  seed {s['seed']}  ({s['scene_kind']})")
    print(f"  features {s['features']}  routes {s['routes']}  issues {s['issues']}  "
          f"route length {s['route_length_m']} m  narrowest route {s['narrowest_route_cm']} cm")
    print("  kinds:  " + ", ".join(f"{k} {v}" for k, v in s["kinds"].items()))
    if s["issues_by_kind"]:
        print("  issues: " + ", ".join(f"{k} {v}" for k, v in s["issues_by_kind"].items()))
    x, y, z = s["suggested_start"]
    print(f"  start: ({x:.0f}, {y:.0f}, {z:.0f}) yaw {s['suggested_start_yaw']:.0f}")


def _print_issues(m: Manifest) -> None:
    for f in m.issues:
        x, y, _ = f.location
        print(f"  {f.kind:10s} {f.subtype:18s} {f.issue:24s} {f.measure:8.2f} / {f.threshold:<8.2f} at ({x:.0f}, {y:.0f})"
              + (f"  on {f.route}" if f.route else ""))


def _print_routes(m: Manifest) -> None:
    for r in m.routes:
        flag = "ok " if r.compliant else "BAD"
        print(f"  {flag} {r.name:24s} {r.length / 100:7.1f} m  nominal {r.nominal_width:6.1f}  min clear {r.min_clear_width:6.1f}  ({len(r.points)} pts)")


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="ramms-worldgen", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)

    def common(parser: argparse.ArgumentParser, top: bool) -> None:
        # The same flags before or after the subcommand; the subparser copies use SUPPRESS so they
        # do not clobber a value given at the top level.
        d = (lambda v: v) if top else (lambda v: argparse.SUPPRESS)
        parser.add_argument("--host", default=d("127.0.0.1"))
        parser.add_argument("--port", type=int, default=d(30010))
        parser.add_argument("--generator", "-g", default=d(""), help="generator actor path, name or label (default: first in the world)")
        parser.add_argument("--timeout", type=float, default=d(180.0), help="seconds to wait for a generation")
        parser.add_argument("--json", action="store_true", default=d(False), help="machine-readable output where it applies")
        parser.add_argument("-v", "--verbose", action="store_true", default=d(False))

    common(p, True)
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("world", help="current map and generator count")
    sub.add_parser("presets", help="list world presets")
    sub.add_parser("maps", help="list generated example maps")
    s = sub.add_parser("open", help="open a map (editor load or game travel)"); s.add_argument("map")
    s = sub.add_parser("spawn", help="spawn a generator"); s.add_argument("--preset", required=True); s.add_argument("--seed", type=int, default=1)
    s.add_argument("--at", default="0,0,0", help="x,y,z"); s.add_argument("--label", default="WorldGenerator")
    s.add_argument("--generate", action="store_true", help="generate right after spawning")
    s = sub.add_parser("generate", help="regenerate"); s.add_argument("--seed", type=int, default=None); s.add_argument("--preset", default="")
    s.add_argument("--no-wait", action="store_true"); s.add_argument("--set", action="append", default=[], metavar="NAME=VALUE")
    s.add_argument("--player-start", action="store_true", help="move the PlayerStart to the suggested start afterwards")
    s = sub.add_parser("randomize", help="new random seed and regenerate"); s.add_argument("--no-wait", action="store_true")
    sub.add_parser("cleanup", help="remove generated content")
    sub.add_parser("status", help="generator status")
    s = sub.add_parser("params", help="show / override parameters"); s.add_argument("--set", action="append", default=[], metavar="NAME=VALUE")
    s.add_argument("--clear", action="store_true", help="drop all per-instance overrides")
    s = sub.add_parser("manifest", help="fetch the ground-truth manifest"); s.add_argument("--out", default="")
    s.add_argument("--issues", action="store_true"); s.add_argument("--routes", action="store_true")
    s.add_argument("--export", action="store_true", help="also have the engine write it under Saved/WorldGen")
    s = sub.add_parser("batch", help="generate many seeds, saving each manifest"); s.add_argument("--preset", required=True)
    s.add_argument("--seeds", required=True, help="e.g. 1-20 or 3,5,8"); s.add_argument("--out", required=True)
    s.add_argument("--set", action="append", default=[], metavar="NAME=VALUE")
    s = sub.add_parser("inspect", help="summarize a saved manifest file (offline)"); s.add_argument("file")
    s.add_argument("--issues", action="store_true"); s.add_argument("--routes", action="store_true")

    for sp in sub.choices.values():
        common(sp, False)
    a = p.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.WARNING, format="%(levelname)s %(name)s: %(message)s")

    def overrides(items: list[str]) -> dict[str, float]:
        out: dict[str, float] = {}
        for it in items:
            if "=" not in it:
                p.error(f"--set expects NAME=VALUE, got {it!r}")
            k, v = it.split("=", 1)
            out[k.strip()] = float(v)
        return out

    if a.cmd == "inspect":
        m = Manifest.load(a.file)
        if a.json:
            print(json.dumps(m.summary(), indent=1))
        else:
            _print_summary(m)
        if a.issues:
            _print_issues(m)
        if a.routes:
            _print_routes(m)
        return 0

    wg = WorldGen(host=a.host, port=a.port, timeout=max(10.0, a.timeout))
    try:
        if not wg.ping():
            print(f"error: no Remote Control API at {a.host}:{a.port} (is the editor running with the plugin enabled?)", file=sys.stderr)
            return 2

        if a.cmd == "world":
            w = wg.world()
            print(json.dumps(w, indent=1) if a.json else f"{w.get('map')}  ({w.get('world_type')})  generators: {w.get('generators')}")
            return 0

        if a.cmd == "presets":
            ps = wg.describe_presets()
            if a.json:
                print(json.dumps(ps, indent=1))
            else:
                for d in ps:
                    print(f"  {d.get('name', ''):22s} {d.get('scene_kind', ''):10s} {d.get('site_size_x', 0) / 100:.0f}x{d.get('site_size_y', 0) / 100:.0f} m   {d.get('path', '')}")
                    if d.get("description"):
                        print(f"      {d['description']}")
            return 0

        if a.cmd == "maps":
            for m in wg.maps():
                print("  " + m)
            return 0

        if a.cmd == "open":
            ok = wg.open_map(a.map)
            print("opened" if ok else "failed")
            return 0 if ok else 1

        if a.cmd == "spawn":
            x, y, z = (float(v) for v in a.at.split(","))
            g = wg.spawn(a.preset, a.seed, (x, y, z), a.label)
            print(g.path)
            if a.generate:
                g.generate(wait=True, timeout=a.timeout)
                _print_summary(g.manifest())
            return 0

        g = wg.generator(a.generator)

        if a.cmd == "generate":
            if a.preset:
                g.set_preset(a.preset)
            if a.set:
                res = g.set_params(overrides(a.set))
                if res.get("unknown"):
                    print(f"warning: unknown parameters ignored: {', '.join(res['unknown'])}", file=sys.stderr)
            st = g.generate(seed=a.seed, wait=not a.no_wait, timeout=a.timeout)
            if a.no_wait:
                print(json.dumps(st, indent=1) if a.json else f"started (seed {st.get('seed')})")
                return 0
            if a.player_start:
                g.place_player_start()
            m = g.manifest()
            print(json.dumps(m.summary(), indent=1) if a.json else "", end="")
            if not a.json:
                _print_summary(m)
            return 0

        if a.cmd == "randomize":
            st = g.randomize(wait=not a.no_wait, timeout=a.timeout)
            if a.no_wait:
                print(f"started (seed {st.get('seed')})")
            else:
                _print_summary(g.manifest())
            return 0

        if a.cmd == "cleanup":
            g.cleanup()
            print("cleaned")
            return 0

        if a.cmd == "status":
            st = g.status()
            if a.json:
                print(json.dumps(st, indent=1))
            else:
                print(f"{st.get('label') or st.get('name')}  preset {st.get('preset', '').rsplit('/', 1)[-1]}  seed {st.get('seed')}"
                      f"  {'GENERATING' if st.get('generating') else 'idle'}  features {st.get('features')}  routes {st.get('routes')}  issues {st.get('issues')}")
                if st.get("override_keys"):
                    print("  overrides: " + ", ".join(st["override_keys"]))
            return 0

        if a.cmd == "params":
            if a.clear:
                g.clear_overrides()
            if a.set:
                res = g.set_params(overrides(a.set))
                if res.get("unknown"):
                    print(f"warning: unknown parameters ignored: {', '.join(res['unknown'])}", file=sys.stderr)
            prm = g.params()
            if a.json:
                print(json.dumps(prm, indent=1, sort_keys=True))
            else:
                for k in sorted(prm):
                    print(f"  {k:26s} {prm[k]}")
            return 0

        if a.cmd == "manifest":
            m = g.manifest()
            if a.out:
                print(f"wrote {m.save(a.out)}")
            if a.export:
                print(f"engine wrote {g.export_manifest('')}")
            if a.json and not a.out:
                print(json.dumps(m.raw))
            elif not a.out:
                _print_summary(m)
            if a.issues:
                _print_issues(m)
            if a.routes:
                _print_routes(m)
            return 0

        if a.cmd == "batch":
            seeds = _seeds(a.seeds)
            n = 0
            for seed, path in wg.batch(a.preset, seeds, a.out, generator=g, params=overrides(a.set) or None, timeout=a.timeout):
                n += 1
                print(f"  [{n}/{len(seeds)}] seed {seed} -> {path}")
            print(f"done: {n} manifests in {a.out}")
            return 0

    except (WorldGenError, UnrealRemoteError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
