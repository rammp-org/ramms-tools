#!/usr/bin/env python3
"""
ramms-worldgen — drive RammsWorldGen (procedural pedestrian / wheelchair worlds)
in a running editor or game over the Remote Control API.

    ramms-worldgen world                              # which map is open, how many generators
    ramms-worldgen presets                            # list presets (name, kind, site size)
    ramms-worldgen palettes                           # list palettes (PAL_Default, PAL_UrbanVerse, ...)
    ramms-worldgen maps                               # list the generated example maps
    ramms-worldgen open Map_Gen_Suburban              # load / travel to a map
    ramms-worldgen spawn --preset PRE_Downtown --seed 7
    ramms-worldgen generate --seed 42                 # regenerate the (first) generator, wait, print summary
    ramms-worldgen generate --seed 42 --palette PAL_UrbanVerse   # same world, UrbanVerse props and cars
    ramms-worldgen generate --palette ""              # back to the preset's palette
    ramms-worldgen randomize
    ramms-worldgen status
    ramms-worldgen params                             # resolved parameter block
    ramms-worldgen params --set HazardDensity=6 --set NonCompliance=0.5 --set FacadeStyles=dt01,dt03
    ramms-worldgen params --clear
    ramms-worldgen glass --set GlassVariation=0.5 --set GlassFrostChance=0.6   # re-roll the glass looks only
    ramms-worldgen manifest --out downtown_42.json    # fetch the manifest
    ramms-worldgen manifest --issues                  # list non-compliant features
    ramms-worldgen manifest --routes                  # list routes with clear widths
    ramms-worldgen batch --preset PRE_Park --seeds 1-50 --out data/park
    ramms-worldgen batch --preset PRE_Downtown --seeds 1-20 --palette PAL_UrbanVerse --out data/dt_uv
    ramms-worldgen inspect data/park/PRE_Park_3.json  # summarize a saved manifest (no engine needed)
    ramms-worldgen terrain                            # graded-street and roughness parameters now
    ramms-worldgen terrain --max-grade 0.08 --cross-slope 0.02 --curb-min 10 --curb-max 20 --generate
    ramms-worldgen terrain --roughness 0.5 --potholes 2 --humps 0.3 --flat   # smooth hills off, rough ground on
    ramms-worldgen manifest --terrain --traffic       # grades, cross slopes, curb heights, roughness; roads, signals
    ramms-worldgen traffic status                     # spawners, cars, signal phase (a traffic level, in play)
    ramms-worldgen traffic spawn | despawn | cars | signals

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
from ramms_tools.worldgen import ROUGHNESS_PARAMS, TERRAIN_PARAMS, Traffic, TrafficError, WorldGen, WorldGenError, terrain_values
from ramms_tools.worldgen.manifest import Manifest

# `terrain` flags -> set_terrain's friendly names (and so the README's parameter names).
TERRAIN_FLAGS = {
    "--max-grade": ("max_grade", "steepest street grade, rise/run (MaxStreetGrade, 0..0.15)"),
    "--crown": ("crown", "road crown slope (RoadCrown, 0..0.08)"),
    "--cross-slope": ("cross_slope", "sidewalk cross slope (SidewalkCrossSlope, 0..0.08)"),
    "--noncompliant-cross-slope": ("noncompliant_cross_slope", "share of blocks with a 3-6 %% cross slope (NonCompliantCrossSlopeChance, 0..1)"),
    "--curb-height": ("curb_height", "every curb, cm, when --curb-max is 0 (CurbHeight)"),
    "--curb-min": ("curb_min", "lowest per-block curb height, cm (CurbHeightMin; never under 2)"),
    "--curb-max": ("curb_max", "highest per-block curb height, cm (CurbHeightMax; 0 = every curb CurbHeight)"),
    "--roughness": ("roughness", "panel lips (SurfaceRoughness, 0..1)"),
    "--panel-length": ("panel_length", "sidewalk panel length, cm (PanelLength)"),
    "--heaves": ("heaves", "tilted panels per 100 m of side (HeaveDensity)"),
    "--cracks": ("cracks", "cracked panels per 100 m (CrackDensity)"),
    "--covers": ("covers", "utility covers per 100 m (CoverDensity)"),
    "--paving": ("paving", "brick / setts patches per 100 m (PavingPatchDensity)"),
    "--potholes": ("potholes", "potholes per 100 m of road (PotholeDensity)"),
    "--humps": ("humps", "speed hump chance per street segment (SpeedHumpChance, 0..1)"),
    "--root-heaves": ("root_heaves", "root heave chance per street tree in a pit (RootHeaveChance, 0..1)"),
}


def _value(v: str) -> float | str:
    """--set NAME=VALUE: a number (true/false as 1/0) when it parses as one, else the text (Palette, FacadeStyles)."""
    t = v.strip()
    if t.lower() in ("true", "false"):
        return 1.0 if t.lower() == "true" else 0.0
    try:
        return float(t)
    except ValueError:
        return t


def _overrides(items: list[str]) -> dict[str, float | str]:
    out: dict[str, float | str] = {}
    for it in items:
        if "=" not in it:
            raise ValueError(f"--set expects NAME=VALUE, got {it!r}")
        k, v = it.split("=", 1)
        out[k.strip()] = _value(v)
    return out


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
    pal = s.get("palette", "").rsplit("/", 1)[-1].split(".")[-1]
    print(f"{s['preset']}  seed {s['seed']}  ({s['scene_kind']})" + (f"  palette {pal}" if pal else ""))
    print(f"  features {s['features']}  routes {s['routes']}  issues {s['issues']}  "
          f"route length {s['route_length_m']} m  narrowest route {s['narrowest_route_cm']} cm")
    print("  kinds:  " + ", ".join(f"{k} {v}" for k, v in s["kinds"].items()))
    if any(s.get("asset_sources", {})):
        print("  assets: " + ", ".join(f"{k or 'palette'} {v}" for k, v in s["asset_sources"].items()))
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


def _print_terrain(m: Manifest) -> None:
    t = m.terrain_summary()
    if not t["graded"]:
        print("  terrain: flat (no Grade records)")
    else:
        lo, hi = t["cross_slope_range"]
        print(f"  terrain: {t['grades']} grades, steepest running grade {t['max_running_grade'] * 100:.1f} %, "
              f"cross slopes {lo * 100:.1f}..{hi * 100:.1f} %  ({t['steep_running_grades']} running / "
              f"{t['steep_cross_slopes']} cross over the limit)")
    if t["curbs"]:
        c0, c1 = t["curb_height_range"]
        print(f"  curbs:   {t['curbs']} sides, {c0:.1f}..{c1:.1f} cm  ("
              + ", ".join(f"{k} {v}" for k, v in t["curbs_by_subtype"].items()) + ")")
    if t["curb_ramps"]:
        print("  ramps:   " + ", ".join(f"{k} {v}" for k, v in t["curb_ramps"].items()))
    if t["roughness"]:
        print("  rough:   " + ", ".join(f"{k} {v}" for k, v in t["roughness"].items()))


def _print_traffic(m: Manifest) -> None:
    t = m.traffic_summary()
    if not t["roads"]:
        print("  traffic: no Road records (not a street scene, or a manifest from before they were recorded)")
        return
    w0, w1 = t["road_width_range_cm"]
    print(f"  roads:   {t['roads']} segments, {t['road_length_m']} m, {w0 / 100:.1f}..{w1 / 100:.1f} m wide")
    print("  inters.: " + (", ".join(f"{k} {v}" for k, v in t["intersections"].items()) or "none")
          + f"   signals {t['signals']}   crosswalks {t['crosswalks']}   humps {t['speed_humps']}   potholes {t['potholes']}")


def _terrain_flags(a: argparse.Namespace) -> dict:
    return {friendly: getattr(a, friendly) for friendly, _ in TERRAIN_FLAGS.values() if getattr(a, friendly) is not None}


def _traffic(a: argparse.Namespace, wg: WorldGen) -> int:
    t = Traffic(wg.ue)
    if a.action == "status":
        st = t.status()
        if a.json:
            print(json.dumps(st, indent=1))
        elif not st["spawners"] and not st["clocks"]:
            print("no traffic in this world (a traffic level, in PIE or a game)")
        else:
            print(f"spawners {st['spawners']}  cars {st['cars']}  signal clocks {st['clocks']}"
                  + (f"  controlled crossings {st.get('controlled_crossings', 0)}" if st["clocks"] else ""))
            if "signals" in st:
                sig = st["signals"]
                print(f"  X {sig['light_x']} ({sig['seconds_to_change_x']:.1f} s)  Y {sig['light_y']} "
                      f"({sig['seconds_to_change_y']:.1f} s)   walk along X {sig['walk_along_x']}, along Y {sig['walk_along_y']}")
        return 0
    if a.action == "spawn":
        res = t.spawn(a.spawner)
        for path, ok in res.items():
            print(f"  {path.rsplit('.', 1)[-1]}: {'spawned' if ok else 'not yet (no lanes, or busy): it spawns on its next tick'}")
        print(f"cars: {t.count()}")
        return 0 if any(res.values()) else 1
    if a.action == "despawn":
        n = t.despawn(a.spawner)
        print(f"despawned ({n} spawner{'s' if n != 1 else ''})")
        return 0
    if a.action == "cars":
        cars = t.cars(limit=a.limit)
        if a.json:
            print(json.dumps([{"name": c.name, "path": c.path, "location": c.location, "rotation": c.rotation} for c in cars], indent=1))
        else:
            for c in cars:
                x, y, z = c.location
                print(f"  {c.name:36s} ({x:9.0f}, {y:9.0f}, {z:6.0f})  yaw {c.yaw:6.1f}")
            print(f"{len(cars)} cars")
        return 0
    if a.action == "signals":
        s = t.signal_state(a.clock)
        if s is None:
            print("no signal clock in this world")
            return 1
        print(json.dumps(s.as_dict(), indent=1) if a.json else str(s))
        return 0
    return 0


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
    sub.add_parser("palettes", help="list world palettes")
    sub.add_parser("maps", help="list generated example maps")
    s = sub.add_parser("open", help="open a map (editor load or game travel)"); s.add_argument("map")
    s = sub.add_parser("spawn", help="spawn a generator"); s.add_argument("--preset", required=True); s.add_argument("--seed", type=int, default=1)
    s.add_argument("--at", default="0,0,0", help="x,y,z"); s.add_argument("--label", default="WorldGenerator")
    s.add_argument("--palette", default=None, help="palette name or path (default: the preset's)")
    s.add_argument("--generate", action="store_true", help="generate right after spawning")
    s = sub.add_parser("generate", help="regenerate"); s.add_argument("--seed", type=int, default=None); s.add_argument("--preset", default="")
    s.add_argument("--palette", default=None, help='palette name or path; "" = the preset\'s')
    s.add_argument("--no-wait", action="store_true"); s.add_argument("--set", action="append", default=[], metavar="NAME=VALUE")
    s.add_argument("--player-start", action="store_true", help="move the PlayerStart to the suggested start afterwards")
    s = sub.add_parser("randomize", help="new random seed and regenerate"); s.add_argument("--no-wait", action="store_true")
    sub.add_parser("cleanup", help="remove generated content")
    sub.add_parser("status", help="generator status")
    s = sub.add_parser("params", help="show / override parameters"); s.add_argument("--set", action="append", default=[], metavar="NAME=VALUE")
    s.add_argument("--clear", action="store_true", help="drop all per-instance overrides")
    s = sub.add_parser("glass", help="re-roll the per-placement glass look (frost, tint, ripple) without regenerating")
    s.add_argument("--set", action="append", default=[], metavar="NAME=VALUE", help="Glass* parameters to override first")
    s = sub.add_parser("manifest", help="fetch the ground-truth manifest"); s.add_argument("--out", default="")
    s.add_argument("--issues", action="store_true"); s.add_argument("--routes", action="store_true")
    s.add_argument("--terrain", action="store_true", help="grades, cross slopes, curb heights, curb ramps, rough ground")
    s.add_argument("--traffic", action="store_true", help="roads, intersections, signals")
    s.add_argument("--export", action="store_true", help="also have the engine write it under Saved/WorldGen")
    s = sub.add_parser("batch", help="generate many seeds, saving each manifest"); s.add_argument("--preset", required=True)
    s.add_argument("--seeds", required=True, help="e.g. 1-20 or 3,5,8"); s.add_argument("--out", required=True)
    s.add_argument("--set", action="append", default=[], metavar="NAME=VALUE")
    s.add_argument("--palette", default="", help="palette name or path (default: the preset's)")
    s.add_argument("--keep-overrides", action="store_true", help="keep the generator's existing overrides")
    s = sub.add_parser("inspect", help="summarize a saved manifest file (offline)"); s.add_argument("file")
    s.add_argument("--issues", action="store_true"); s.add_argument("--routes", action="store_true")
    s.add_argument("--terrain", action="store_true"); s.add_argument("--traffic", action="store_true")
    s = sub.add_parser("terrain", help="show / set the graded-street and rough-ground parameters")
    for flag, (dest, help_) in TERRAIN_FLAGS.items():
        s.add_argument(flag, dest=dest, type=float, default=None, help=help_)
    s.add_argument("--flat", action="store_true", help="zero every terrain and roughness parameter first")
    s.add_argument("--generate", action="store_true", help="regenerate afterwards and summarize the ground")
    s.add_argument("--seed", type=int, default=None, help="with --generate")
    s = sub.add_parser("traffic", help="moving traffic: status, spawn, despawn, cars, signals")
    s.add_argument("action", choices=["status", "spawn", "despawn", "cars", "signals"])
    s.add_argument("--spawner", default="", help="one spawner by name (default: all)")
    s.add_argument("--clock", default="", help="one signal clock by name (default: the first)")
    s.add_argument("--limit", type=int, default=0, help="cars: read at most this many")

    for sp in sub.choices.values():
        common(sp, False)
    a = p.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.WARNING, format="%(levelname)s %(name)s: %(message)s")

    def overrides(items: list[str]) -> dict[str, float | str]:
        try:
            return _overrides(items)
        except ValueError as e:
            p.error(str(e))

    if a.cmd == "inspect":
        m = Manifest.load(a.file)
        if a.json:
            out = m.summary()
            if a.terrain:
                out["terrain"] = m.terrain_summary()
            if a.traffic:
                out["traffic"] = m.traffic_summary()
            print(json.dumps(out, indent=1))
        else:
            _print_summary(m)
            if a.terrain:
                _print_terrain(m)
            if a.traffic:
                _print_traffic(m)
        if a.issues:
            _print_issues(m)
        if a.routes:
            _print_routes(m)
        return 0

    if a.cmd == "terrain":
        try:
            values = terrain_values(flat=a.flat, **_terrain_flags(a))
        except (KeyError, ValueError) as e:
            p.error(str(e.args[0] if e.args else e))

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

        if a.cmd == "palettes":
            pals = wg.palettes()
            if a.json:
                print(json.dumps(pals, indent=1))
            else:
                for pp in pals:
                    print(f"  {pp.rsplit('/', 1)[-1].split('.')[-1]:22s} {pp}")
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
            g = wg.spawn(a.preset, a.seed, (x, y, z), a.label, palette=a.palette or "")
            print(g.path)
            if a.generate:
                g.generate(wait=True, timeout=a.timeout)
                _print_summary(g.manifest())
            return 0

        if a.cmd == "traffic":
            return _traffic(a, wg)

        if a.cmd == "batch":
            seeds = _seeds(a.seeds)
            # No generator in the world is fine for a batch: it spawns one.
            g = wg.generator(a.generator) if (a.generator or wg.generators()) else None
            n = 0
            try:
                for seed, path in wg.batch(a.preset, seeds, a.out, generator=g, params=overrides(a.set) or None, timeout=a.timeout,
                                           palette=a.palette, keep_overrides=a.keep_overrides):
                    n += 1
                    print(f"  [{n}/{len(seeds)}] seed {seed} -> {path}")
            except KeyError as e:
                print(f"error: unknown parameters: {e.args[0]}", file=sys.stderr)
                return 1
            print(f"done: {n} manifests in {a.out}")
            return 0

        g = wg.generator(a.generator)

        if a.cmd == "terrain":
            if values:
                res = g.set_params(values)
                if res.get("unknown"):
                    print(f"warning: this RammsWorldGen has no {', '.join(res['unknown'])} (ignored)", file=sys.stderr)
            if a.generate:
                g.generate(seed=a.seed, wait=True, timeout=a.timeout)
            cur = g.terrain()
            if a.json:
                out = {"params": cur}
                if a.generate:
                    out["terrain"] = g.manifest().terrain_summary()
                print(json.dumps(out, indent=1))
                return 0
            names = {friendly: param for friendly, (param, _, _) in {**TERRAIN_PARAMS, **ROUGHNESS_PARAMS}.items()}
            for friendly, value in cur.items():
                print(f"  {friendly:26s} {names[friendly]:30s} {value}")
            if not cur:
                print("  (this RammsWorldGen has no terrain parameters)")
            if a.generate:
                m = g.manifest()
                _print_summary(m)
                _print_terrain(m)
            elif values:
                print("(takes effect on the next generate; --generate to do it now)")
            return 0

        if a.cmd == "glass":
            if a.set:
                res = g.set_params(overrides(a.set))
                if res.get("unknown"):
                    print(f"warning: unknown parameters ignored: {', '.join(res['unknown'])}", file=sys.stderr)
            n = g.vary_glass()
            glass = {k: v for k, v in g.params().items() if k.startswith("Glass")}
            print(json.dumps({"instances": n, **glass}, indent=1) if a.json else
                  f"{n} glass instances varied  (" + ", ".join(f"{k} {v}" for k, v in glass.items()) + ")")
            return 0

        if a.cmd == "generate":
            if a.preset:
                g.set_preset(a.preset)
            if a.palette is not None:
                g.set_palette(a.palette)
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
                pal = st.get("palette", "").rsplit("/", 1)[-1].split(".")[-1]
                print(f"{st.get('label') or st.get('name')}  preset {st.get('preset', '').rsplit('/', 1)[-1]}  palette {pal or '-'}  seed {st.get('seed')}"
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
                if a.terrain:
                    _print_terrain(m)
                if a.traffic:
                    _print_traffic(m)
            if a.issues:
                _print_issues(m)
            if a.routes:
                _print_routes(m)
            return 0

    except (WorldGenError, TrafficError, UnrealRemoteError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
