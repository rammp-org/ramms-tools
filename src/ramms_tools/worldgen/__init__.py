"""
Drive RammsWorldGen (procedural pedestrian / wheelchair worlds) from outside
the engine over the Remote Control API.

The UE side is the ``RammsWorldGen`` plugin: an ``ARammsWorldGenerator`` actor
in the level runs a PCG graph for a preset + seed and publishes a ground-truth
manifest. ``URammsWorldGenBridge`` (a function library CDO) exposes it to
Remote Control with plain strings and numbers, and this module wraps that.

    from ramms_tools.worldgen import WorldGen

    wg = WorldGen()                                  # localhost:30010
    print(wg.world())                                # {'map': 'Map_Gen_Downtown', 'play': False, ...}
    gen = wg.generator()                             # first generator in the level
    gen.generate(seed=42, wait=True)                 # regenerate and block until the manifest is in
    m = gen.manifest()                               # ramms_tools.worldgen.manifest.Manifest
    print(m.summary())
    for f in m.issues:
        print(f.kind, f.subtype, f.issue, f.location)

    # dataset: twenty downtown worlds, manifests on disk
    for seed, path in wg.batch("PRE_Downtown", range(20), out_dir="data/downtown"):
        print(seed, path)

Everything here also works against a running game (-game or packaged with the
Remote Control plugin enabled), since the generator and the bridge are runtime
code; only ``open_map`` behaves differently (travel vs. editor load).
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Iterable, Iterator, Optional

from ramms_tools.unreal_remote import UnrealRemote, UnrealRemoteError
from .manifest import Feature, Manifest, Route

__all__ = ["WorldGen", "Generator", "Manifest", "Feature", "Route", "WorldGenError", "BRIDGE"]

BRIDGE = "/Script/RammsWorldGen.Default__RammsWorldGenBridge"


class WorldGenError(UnrealRemoteError):
    pass


def _as_list(result: Any) -> list:
    if isinstance(result, list):
        return result
    if isinstance(result, dict):
        for k in ("ReturnValue", "result"):
            if isinstance(result.get(k), list):
                return result[k]
    return []


def _as_json(result: Any) -> Any:
    """Bridge functions return JSON in a string; Remote Control may or may not wrap it."""
    if isinstance(result, dict) and "ReturnValue" in result:
        result = result["ReturnValue"]
    if isinstance(result, str):
        return json.loads(result) if result else None
    return result


class Generator:
    """One ARammsWorldGenerator actor, addressed by actor path."""

    def __init__(self, wg: "WorldGen", path: str):
        self.wg = wg
        self.path = path

    def __repr__(self) -> str:
        return f"Generator({self.path!r})"

    # ── lifecycle ───────────────────────────────────────────────────

    def generate(self, seed: Optional[int] = None, wait: bool = True, timeout: float = 120.0) -> dict:
        """Regenerate (with a new seed when given). With wait=True, block until the manifest is published."""
        if not self.wg._call("Generate", GeneratorPath=self.path, Seed=-1 if seed is None else int(seed)):
            raise WorldGenError(f"generator not found: {self.path}")
        return self.wait(timeout) if wait else self.status()

    def randomize(self, wait: bool = True, timeout: float = 120.0) -> dict:
        if not self.wg._call("Randomize", GeneratorPath=self.path):
            raise WorldGenError(f"generator not found: {self.path}")
        return self.wait(timeout) if wait else self.status()

    def cleanup(self) -> None:
        self.wg._call("Cleanup", GeneratorPath=self.path)

    def is_generating(self) -> bool:
        return bool(self.wg._call("IsGenerating", GeneratorPath=self.path))

    def wait(self, timeout: float = 120.0, poll: float = 0.25) -> dict:
        """Poll until generation finishes; returns the final status. Raises on timeout."""
        t0 = time.monotonic()
        # Generation is scheduled on the next tick, so give it a moment to start before trusting "not generating".
        time.sleep(min(poll, 0.2))
        while True:
            st = self.status()
            if not st.get("generating") and st.get("has_manifest") and st.get("manifest_seed") == st.get("seed"):
                return st
            if not st.get("generating") and time.monotonic() - t0 > 2.0 and not st.get("has_manifest"):
                # Nothing running and nothing produced: the generator is idle (no preset / graph?).
                return st
            if time.monotonic() - t0 > timeout:
                raise WorldGenError(f"generation did not finish within {timeout:.0f}s: {st}")
            time.sleep(poll)

    def place_player_start(self) -> bool:
        return bool(self.wg._call("PlacePlayerStart", GeneratorPath=self.path))

    # ── state ───────────────────────────────────────────────────────

    def status(self) -> dict:
        st = _as_json(self.wg._call("GetStatusJson", GeneratorPath=self.path)) or {}
        if not st.get("ok", False):
            raise WorldGenError(st.get("error", f"generator not found: {self.path}"))
        return st

    def manifest(self) -> Manifest:
        raw = self.wg._call("GetManifestJson", GeneratorPath=self.path)
        if isinstance(raw, dict) and "ReturnValue" in raw:
            raw = raw["ReturnValue"]
        if not raw:
            raise WorldGenError(f"no manifest from {self.path}")
        return Manifest.from_json(raw)

    def export_manifest(self, path: str | Path = "") -> str:
        """Write the manifest on the engine's machine ("" = Saved/WorldGen/<auto>.json). Returns the path written there."""
        out = self.wg._call("ExportManifest", GeneratorPath=self.path, AbsolutePath=str(path))
        if isinstance(out, dict) and "ReturnValue" in out:
            out = out["ReturnValue"]
        if not out:
            raise WorldGenError("export failed")
        return str(out)

    # ── parameters ──────────────────────────────────────────────────

    @property
    def preset(self) -> str:
        return self.status().get("preset", "")

    def set_preset(self, preset: str) -> None:
        if not self.wg._call("SetPreset", GeneratorPath=self.path, Preset=preset):
            raise WorldGenError(f"preset not found or generator missing: {preset}")

    @property
    def seed(self) -> int:
        return int(self.status().get("seed", 0))

    def params(self) -> dict[str, float]:
        """The resolved parameter block (preset + profile + overrides)."""
        return _as_json(self.wg._call("GetParamsJson", GeneratorPath=self.path)) or {}

    def set_params(self, values: Optional[dict[str, float]] = None, **kwargs: float) -> dict:
        """
        Override parameters by name (see the plugin README for the list). Takes effect on the next generate().
        Returns {"applied": [...], "unknown": [...]}.
        """
        merged = dict(values or {})
        merged.update(kwargs)
        res = _as_json(self.wg._call("SetParamsJson", GeneratorPath=self.path, Json=json.dumps(merged))) or {}
        if not res.get("ok", False):
            raise WorldGenError(res.get("error", "set_params failed"))
        return res

    def clear_overrides(self) -> None:
        self.wg._call("ClearOverrides", GeneratorPath=self.path)

    def __getitem__(self, name: str) -> float:
        p = self.params()
        if name not in p:
            raise KeyError(name)
        return p[name]

    def __setitem__(self, name: str, value: float) -> None:
        res = self.set_params({name: value})
        if name in res.get("unknown", []):
            raise KeyError(name)


class WorldGen:
    """
    Entry point. Wraps an UnrealRemote and the RammsWorldGenBridge CDO.

    Args:
        ue: an existing UnrealRemote, or None to connect to host:port.
    """

    def __init__(self, ue: Optional[UnrealRemote] = None, host: str = "127.0.0.1", port: int = 30010,
                 timeout: float = 30.0):
        self.ue = ue or UnrealRemote(host=host, http_port=port, timeout=timeout)

    def _call(self, func: str, **params: Any) -> Any:
        return self.ue._call_function(BRIDGE, func, params if params else None)

    # ── discovery ───────────────────────────────────────────────────

    def ping(self) -> bool:
        return self.ue.ping()

    def world(self) -> dict:
        """{"map", "package", "play", "world_type", "generators"} for the current world."""
        return _as_json(self._call("GetWorldInfo")) or {"ok": False}

    def presets(self) -> list[str]:
        """Asset paths of every world preset."""
        return [str(p) for p in _as_list(self._call("ListPresets"))]

    def describe_presets(self) -> list[dict]:
        """[{"name", "path", "scene_kind", "description", "site_size_x", "site_size_y", "palette", "graph"}]"""
        return _as_json(self._call("DescribePresets")) or []

    def maps(self) -> list[str]:
        """Package paths of the generated example maps (/Game/Maps/Gen/Map_Gen_*)."""
        return [str(p) for p in _as_list(self._call("ListGeneratedMaps"))]

    def generators(self) -> list[Generator]:
        return [Generator(self, str(p)) for p in _as_list(self._call("FindGenerators"))]

    def generator(self, path_or_name: str = "") -> Generator:
        """A generator by actor path, name or label; "" = the first one in the world."""
        if path_or_name:
            g = Generator(self, path_or_name)
            g.status()  # validates
            return g
        gens = self.generators()
        if not gens:
            raise WorldGenError("no RammsWorldGenerator in the current world (spawn one with WorldGen.spawn)")
        return gens[0]

    # ── lifecycle ───────────────────────────────────────────────────

    def spawn(self, preset: str, seed: int = 1, location: tuple[float, float, float] = (0.0, 0.0, 0.0),
              label: str = "WorldGenerator") -> Generator:
        """Spawn a generator in the current world pointed at a preset (asset path or short name). Does not generate."""
        x, y, z = location
        path = self._call("SpawnGenerator", Preset=preset, Seed=int(seed), Location={"X": x, "Y": y, "Z": z}, Label=label)
        if isinstance(path, dict) and "ReturnValue" in path:
            path = path["ReturnValue"]
        if not path:
            raise WorldGenError(f"could not spawn a generator for preset {preset!r}")
        return Generator(self, str(path))

    def open_map(self, map_name: str, settle: float = 0.0) -> bool:
        """
        Open a map by short name or package path. In a game this travels; in the editor it loads the level.
        After a travel the old actor paths are invalid — call generators() again.
        """
        ok = bool(self._call("OpenMap", Map=map_name))
        if ok and settle > 0:
            time.sleep(settle)
        return ok

    # ── dataset helpers ─────────────────────────────────────────────

    def batch(self, preset: str, seeds: Iterable[int], out_dir: str | Path, generator: Optional[Generator] = None,
              params: Optional[dict[str, float]] = None, timeout: float = 180.0,
              export_on_engine: bool = False) -> Iterator[tuple[int, Path]]:
        """
        Generate one world per seed and save each manifest under out_dir as <preset>_<seed>.json.

        The manifests are fetched over the API and written locally (so this works against a remote
        engine); with export_on_engine=True the engine writes them too (its Saved/WorldGen).
        Yields (seed, local_path) as each world finishes; the last world stays in the level.
        """
        out = Path(out_dir)
        out.mkdir(parents=True, exist_ok=True)
        gen = generator or (self.generators()[0] if self.generators() else self.spawn(preset, 0))
        gen.set_preset(preset)
        if params:
            gen.set_params(params)
        short = preset.rsplit("/", 1)[-1].split(".")[-1]
        for seed in seeds:
            gen.generate(seed=int(seed), wait=True, timeout=timeout)
            m = gen.manifest()
            p = m.save(out / f"{short}_{int(seed)}.json")
            if export_on_engine:
                gen.export_manifest("")
            yield int(seed), p
