#!/usr/bin/env python3
"""
Live smoke test for ramms_tools.worldgen against a running editor (PIE or not)
or a packaged / -game build with Remote Control enabled.

    python examples/worldgen_smoke.py                 # localhost:30010
    python examples/worldgen_smoke.py --host 10.0.0.5 --map Map_Gen_Store

Checks discovery, generation, determinism (same seed -> same manifest), parameter
overrides (numbers and text), palettes (switching to another one and back, e.g. the
UrbanVerse-100K palette when it is imported), per-placement glass variation, the assets recorded in the manifest,
manifest export / reload, a small batch, and map travel. Exits non-zero on the first failure.
"""

from __future__ import annotations

import argparse
import sys
import tempfile
import time
from pathlib import Path

from ramms_tools.worldgen import WorldGen, WorldGenError
from ramms_tools.worldgen.manifest import Manifest


def _ok(msg: str) -> None:
    print(f"  PASS  {msg}", flush=True)


def _fail(msg: str) -> None:
    print(f"  FAIL  {msg}", flush=True)
    sys.exit(1)


def _check(cond: bool, msg: str) -> None:
    _ok(msg) if cond else _fail(msg)


def _brief(m: Manifest) -> str:
    s = m.summary()
    return (f"{s['preset']} seed {s['seed']}: {s['features']} features, {s['routes']} routes, "
            f"{s['issues']} issues, {s['route_length_m']} m of route")


def _key(m: Manifest) -> tuple:
    s = m.summary()
    return (s["features"], s["routes"], s["issues"], s["route_length_m"])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=30010)
    ap.add_argument("--map", default="Map_Gen_Store", help="map to travel to at the end ('' to skip)")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--palette", default="PAL_UrbanVerse", help="second palette to switch to ('' to skip; skipped if not imported)")
    args = ap.parse_args()

    t0 = time.time()
    wg = WorldGen(host=args.host, port=args.port)
    print(f"ramms_tools.worldgen smoke test -> {args.host}:{args.port}", flush=True)

    _check(wg.ping(), "Remote Control reachable")
    w = wg.world()
    print(f"        world: {w}")
    _check(bool(w.get("map")), f"world info ({w.get('world_type')}, map {w.get('map')})")

    presets = wg.describe_presets()
    names = [p.get("name") for p in presets]
    print(f"        presets: {names}")
    _check(len(presets) >= 7 and "PRE_Downtown" in names, f"{len(presets)} presets discovered")
    maps = wg.maps()
    _check(len(maps) >= 1, f"{len(maps)} generated maps listed")

    gen = wg.generator()
    st = gen.status()
    print(f"        generator: {gen.path}  preset {st.get('preset')}  seed {st.get('seed')}")
    _check(bool(st), "generator found and status read")

    gen.generate(seed=args.seed, wait=True)
    a = gen.manifest()
    print(f"        {_brief(a)}")
    _check(len(a.features) > 0 and len(a.routes) > 0, f"seed {args.seed} generated ({len(a.features)} features, {len(a.routes)} routes)")

    if a.features_of("Crosswalk") or str(a.scene_kind).lower() not in ("indoor", ""):
        # Street scenes: every crossing that joins two sidewalks is recorded, and a step-free one has a
        # curb ramp at each end (the ramp's top sits a ramp run, about 1.8 m, back from the curb).
        cws = a.crosswalks()
        free = a.crosswalks(step_free=True)
        missing = [f for f in free if not all(any(r.kind == "CurbRamp" for r in a.features_near(e, 300.0)) for e in f.crossing_ends())]
        print(f"        crosswalks: {len(cws)} ({len(free)} step-free), subtypes {sorted({f.subtype for f in cws})}")
        _check(len(cws) > 0 and len(free) > 0 and not missing, f"{len(free)} step-free crosswalks, each with a curb ramp at both ends")

    gen.generate(seed=args.seed + 1, wait=True)
    b = gen.manifest()
    print(f"        {_brief(b)}")
    _check(_key(a) != _key(b), f"seed {args.seed + 1} gives a different world")

    gen.generate(seed=args.seed, wait=True)
    a2 = gen.manifest()
    _check(_key(a) == _key(a2), f"seed {args.seed} again gives the same world (deterministic)")

    params = gen.params()
    _check(len(params) > 10, f"{len(params)} parameters readable")
    if "NonCompliance" in params:
        res = gen.set_params(NonCompliance=1.0)
        print(f"        set_params -> {res}")
        gen.generate(seed=args.seed, wait=True)
        c = gen.manifest()
        print(f"        NonCompliance=1.0: {_brief(c)}")
        _check(len(c.issues) >= len(a.issues), f"override applied ({len(c.issues)} issues vs {len(a.issues)} at the preset value)")
        gen.clear_overrides()
        gen.generate(seed=args.seed, wait=True)
        _check(_key(gen.manifest()) == _key(a), "overrides cleared")

    # Text parameters go through set_params as strings.
    if "FacadeStyles" in params:
        res = gen.set_params(FacadeStyles="dt01")
        _check("FacadeStyles" in res.get("applied", []) and gen.params().get("FacadeStyles") == "dt01", "text parameter override (FacadeStyles)")
        gen.clear_overrides()
        _check(gen.params().get("FacadeStyles") == params["FacadeStyles"], "text override cleared")

    # Assets: palette meshes are recorded with their asset path; nothing points at engine placeholder meshes.
    with_asset = [f for f in a.features if f.asset]
    _check(len(with_asset) > 0 and not any(f.asset.startswith("/Engine/") for f in with_asset),
           f"{len(with_asset)} features record their mesh asset ({len(a.assets())} distinct)")
    _check(bool(a.palette), f"manifest records its palette ({a.palette.rsplit('/', 1)[-1]})")

    # Palettes: switch to another one and back.
    pals = wg.palettes()
    short = lambda p: p.rsplit("/", 1)[-1].split(".")[-1]
    print(f"        palettes: {[short(p) for p in pals]}")
    _check(any(short(p) == "PAL_Default" for p in pals), f"{len(pals)} palettes listed")
    try:
        gen.set_palette("PAL_DoesNotExist")
        _fail("unknown palette rejected")
    except WorldGenError:
        _ok("unknown palette rejected")
    if args.palette and any(short(p) == args.palette for p in pals):
        before = gen.palette
        gen.set_palette(args.palette)
        _check(short(gen.palette) == args.palette, f"set_palette({args.palette})")
        gen.generate(seed=args.seed, wait=True)
        u = gen.manifest()
        srcs = u.sources()
        print(f"        {args.palette}: {_brief(u)}, asset sources {srcs}")
        ds = {f.source_dataset for f in u.features if f.source}
        _check(short(u.palette) == args.palette and sum(v for k, v in srcs.items() if k) > 0,
               f"world generated with {args.palette} ({sum(v for k, v in srcs.items() if k)} assets from {sorted(ds)})")
        if "urbanverse" in ds:
            uv = [f for f in u.features if f.source_dataset == "urbanverse"]
            _check(all(f.asset.startswith("/Game/UrbanVerse/") and len(f.source_uid) == 32 for f in uv),
                   f"{len(uv)} UrbanVerse features carry their asset and uid")
        # Glass: each placement gets its own look (cosmetic; the layout and manifest must not change).
        glass = {k: v for k, v in gen.params().items() if k.startswith("Glass")}
        if glass:
            _check(set(glass) >= {"GlassVariation", "GlassFrostChance", "GlassTintChance", "GlassBumpChance"},
                   f"glass parameters readable {glass}")
            n = gen.vary_glass()
            _check(n > 0 or "urbanverse" not in ds, f"vary_glass(): {n} glass instances varied")
            gen.set_params(GlassVariation=0.0, GlassFrostChance=1.0)
            gen.generate(seed=args.seed, wait=True)
            _check(_key(gen.manifest()) == _key(u), "glass parameters leave the layout unchanged")
            _check(gen.vary_glass() == 0, "GlassVariation 0: no instance varied")
            gen.clear_overrides()
            _check(gen.vary_glass() == n, f"overrides cleared: {n} instances varied again")
        gen.set_palette("")
        _check(gen.palette == before, "palette back to the preset's")
        gen.generate(seed=args.seed, wait=True)
        _check(_key(gen.manifest()) == _key(a), "same world again with the preset's palette")
    elif args.palette:
        print(f"        ({args.palette} not imported; palette switch skipped)")

    with tempfile.TemporaryDirectory() as tmp:
        local = gen.manifest().save(Path(tmp) / "smoke.json")
        _check(_key(Manifest.load(local)) == _key(a), "manifest saved and reloaded offline")
        engine_path = gen.export_manifest("")
        _check(bool(engine_path), f"manifest exported on the engine side: {engine_path}")
        preset = st.get("preset") or "PRE_Downtown"
        got = list(wg.batch(preset, [1, 2, 3], out_dir=Path(tmp) / "batch", generator=gen))
        _check(len(got) == 3 and all(p.exists() for _, p in got), f"batch of 3 {preset} worlds written")

    if args.map:
        _check(wg.open_map(args.map, settle=8.0), f"open_map {args.map}")
        for _ in range(30):
            try:
                w = wg.world()
                if w.get("map", "").endswith(args.map) and w.get("generators", 0) > 0:
                    break
            except Exception:
                pass
            time.sleep(1.0)
        _check(w.get("map", "").endswith(args.map), f"now in {w.get('map')}")
        g2 = wg.generator()
        g2.generate(seed=3, wait=True)
        m = g2.manifest()
        print(f"        {args.map} seed 3: {_brief(m)}")
        _check(len(m.features) > 0, f"generated after travel ({len(m.features)} features)")

    print(f"ALL PASSED in {time.time() - t0:.1f} s", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
