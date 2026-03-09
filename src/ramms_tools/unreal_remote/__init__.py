"""
Unreal Engine Remote Control API client.

Provides a Pythonic interface to UE's Remote Control HTTP/WebSocket API.
Supports querying actors/components, getting/setting properties, and calling
functions — all via dynamic proxy objects that translate attribute access
and method calls into remote API requests.

Core features (RammsCore plugin only):
    - Actor and component discovery via URammsCoreBridge
    - Property read/write on any UObject
    - Function calls on any UObject
    - Object description (introspection)

Optional UI features (requires RammsUI plugin):
    - Widget discovery, notifications, status panel control
    - Pass ui_bridge="/Script/RammsUI.Default__RammsRemoteBridge" to enable

Usage:
    from unreal_remote import UnrealRemote

    ue = UnrealRemote()  # connects to localhost:30010

    # List all actors
    actors = ue.find_actors()

    # Find actors by class
    cameras = ue.find_actors(class_filter="CameraActor")

    # Find components on an actor
    comps = ue.find_components(actor_path, "Controller")

    # Get a proxy for a specific actor by path
    actor = ue.actor("/Game/MyMap.MyMap:PersistentLevel.MyActor_0")

    # Read a property
    location = actor.get_property("RootComponent.RelativeLocation")

    # Set a property
    actor.set_property("RootComponent.RelativeLocation", {"X": 100, "Y": 0, "Z": 50})

    # Call a function
    result = actor.call("MyFunction", MyParam=123)

    # Duck-typed method calls (auto-translated to remote calls)
    actor.SetActorLocation(NewLocation={"X": 100, "Y": 0, "Z": 50})
"""

from __future__ import annotations

import json
import time
import logging
from typing import Any, Optional
from urllib.request import Request, urlopen
from urllib.error import URLError, HTTPError

logger = logging.getLogger(__name__)


class UnrealRemoteError(Exception):
    """Raised when a remote control API call fails."""
    pass


class RemoteObjectProxy:
    """
    Proxy for a remote UObject. Attribute access and method calls are
    forwarded to the Unreal Remote Control API.

    Properties:
        proxy.get_property("PropertyName")
        proxy.set_property("PropertyName", value)

    Function calls (duck-punched):
        proxy.MyFunction(Param1=value1, Param2=value2)
        # translates to PUT /remote/object/call with functionName="MyFunction"
    """

    def __init__(self, client: UnrealRemote, object_path: str):
        # Use object.__setattr__ to avoid triggering __setattr__ override
        object.__setattr__(self, '_client', client)
        object.__setattr__(self, '_object_path', object_path)

    @property
    def object_path(self) -> str:
        return object.__getattribute__(self, '_object_path')

    def get_property(self, property_name: str, access: str = "READ_ACCESS") -> Any:
        """Read a property value from this object."""
        client = object.__getattribute__(self, '_client')
        return client._get_property(self.object_path, property_name, access)

    def set_property(self, property_name: str, value: Any) -> Any:
        """Set a property value on this object."""
        client = object.__getattribute__(self, '_client')
        return client._set_property(self.object_path, property_name, value)

    def call(self, function_name: str, **kwargs) -> Any:
        """Call a function on this object with keyword arguments as parameters."""
        client = object.__getattribute__(self, '_client')
        return client._call_function(self.object_path, function_name, kwargs)

    def get_components(self, class_filter: Optional[str] = None) -> list[RemoteObjectProxy]:
        """Get components of this actor, optionally filtered by class name."""
        client = object.__getattribute__(self, '_client')
        comps = client.find_components(self.object_path, class_filter or "")
        return [RemoteObjectProxy(client, c["path"]) for c in comps]

    def __getattr__(self, name: str):
        """
        Duck-punch: unknown attribute access returns a callable that
        translates to a remote function call.

        Usage: proxy.SetActorLocation(NewLocation={"X": 1, "Y": 2, "Z": 3})
        """
        # Return a callable that invokes the remote function
        def _remote_call(**kwargs):
            return self.call(name, **kwargs)
        return _remote_call

    def __setattr__(self, name: str, value: Any):
        """Setting attributes translates to setting remote properties."""
        self.set_property(name, value)

    def __repr__(self) -> str:
        return f"RemoteObjectProxy('{self.object_path}')"


class UnrealRemote:
    """
    Client for the Unreal Engine Remote Control HTTP API.

    Args:
        host: Hostname or IP of the UE instance (default: "127.0.0.1")
        http_port: HTTP API port (default: 30010)
        timeout: Request timeout in seconds (default: 5.0)
    """

    def __init__(self, host: str = "127.0.0.1", http_port: int = 30010,
                 timeout: float = 5.0, ui_bridge: Optional[str] = None):
        self.base_url = f"http://{host}:{http_port}"
        self.timeout = timeout
        if ui_bridge is not None:
            self.ramms_ui_bridge = ui_bridge

    # Well-known object paths
    EDITOR_ACTOR_SUBSYSTEM = "/Script/UnrealEd.Default__EditorActorSubsystem"
    EDITOR_ASSET_LIBRARY = "/Script/EditorScriptingUtilities.Default__EditorAssetLibrary"
    RAMMS_CORE_BRIDGE = "/Script/RammsCore.Default__RammsCoreBridge"

    # Optional: set to the RammsUI bridge CDO path if the RammsUI plugin is installed.
    # Default is None (UI bridge not available).
    ramms_ui_bridge: Optional[str] = None

    # Backward-compat alias
    RAMMS_REMOTE_BRIDGE = RAMMS_CORE_BRIDGE

    # ── High-level API ──────────────────────────────────────────────

    def actor(self, object_path: str) -> RemoteObjectProxy:
        """Get a proxy for an object by its full path."""
        return RemoteObjectProxy(self, object_path)

    @property
    def bridge(self) -> RemoteObjectProxy:
        """Get a proxy for the URammsCoreBridge function library CDO."""
        return RemoteObjectProxy(self, self.RAMMS_CORE_BRIDGE)

    @property
    def ui_bridge(self) -> RemoteObjectProxy:
        """
        Get a proxy for the URammsRemoteBridge (UI-specific) function library CDO.
        Requires the RammsUI plugin to be installed and the ui_bridge path to be set,
        either via the constructor or by setting ramms_ui_bridge on the instance.

        Raises:
            ValueError: If no UI bridge path has been configured.
        """
        if not self.ramms_ui_bridge:
            raise ValueError(
                "No UI bridge configured. Pass ui_bridge='/Script/RammsUI.Default__RammsRemoteBridge' "
                "to UnrealRemote() or set instance.ramms_ui_bridge."
            )
        return RemoteObjectProxy(self, self.ramms_ui_bridge)

    def find_actors(self, class_filter: Optional[str] = None,
                    name_filter: Optional[str] = None) -> list[RemoteObjectProxy]:
        """
        Find actors in the current level.

        Tries the RammsCoreBridge first (uses GEngine->GetWorldContexts()),
        then falls back to EditorActorSubsystem.

        Args:
            class_filter: Class name substring to filter (e.g. "StaticMeshActor").
            name_filter: Substring match on actor path/name (applied client-side).

        Returns:
            List of RemoteObjectProxy for matching actors.
        """
        # Try RammsCoreBridge (works with proper world context via GEngine)
        try:
            if class_filter:
                result = self._call_function(
                    self.RAMMS_CORE_BRIDGE,
                    "FindActors",
                    {"ClassNameFilter": class_filter}
                )
            else:
                result = self._call_function(
                    self.RAMMS_CORE_BRIDGE,
                    "GetAllActorPaths"
                )
            return self._parse_path_list(result, name_filter)
        except UnrealRemoteError as e:
            logger.debug(f"RammsCoreBridge actor search failed: {e}")

        # Fallback: EditorActorSubsystem (only reached if bridge call failed)
        try:
            result = self._call_function(
                self.EDITOR_ACTOR_SUBSYSTEM,
                "GetAllLevelActors"
            )
            actors = self._parse_actor_list(result, name_filter)
            # Apply class_filter client-side since EditorActorSubsystem doesn't support it
            if class_filter:
                actors = [a for a in actors
                          if class_filter.lower() in a.object_path.lower()]
            return actors
        except UnrealRemoteError as e:
            logger.warning(f"EditorActorSubsystem call failed: {e}")
            return []

    def find_ramms_widgets(self, class_filter: str = "") -> list[RemoteObjectProxy]:
        """
        Find live URammsBaseWidget instances using the RammsUI bridge.
        Requires the RammsUI plugin and a configured ui_bridge path.

        Args:
            class_filter: Class name substring (e.g. "StatusPanel", "Toolbar").

        Returns:
            List of RemoteObjectProxy for matching widget instances.
        """
        if not self.ramms_ui_bridge:
            logger.warning("find_ramms_widgets: No UI bridge configured (RammsUI plugin required)")
            return []

        try:
            if class_filter:
                result = self._call_function(
                    self.ramms_ui_bridge,
                    "FindRammsWidgets",
                    {"ClassNameFilter": class_filter}
                )
            else:
                result = self._call_function(
                    self.ramms_ui_bridge,
                    "GetAllRammsWidgetPaths"
                )
            return self._parse_path_list(result)
        except UnrealRemoteError as e:
            logger.warning(f"FindRammsWidgets failed: {e}")
            return []

    def find_actors_by_component(self, component_class_filter: str
                                ) -> list[dict]:
        """
        Find actors that have a component matching a class name substring.

        Returns list of dicts with 'actor_path', 'actor_proxy',
        'component_name', 'component_class', and 'component_path' keys.
        Single server-side call — much faster than iterating all actors.
        """
        try:
            result = self._call_function(
                self.RAMMS_CORE_BRIDGE,
                "FindActorsByComponent",
                {"ComponentClassFilter": component_class_filter}
            )
            if isinstance(result, dict):
                result = result.get("ReturnValue", result)

            entries = []
            if isinstance(result, list):
                for entry in result:
                    if isinstance(entry, str) and "|" in entry:
                        actor_path, comp_info = entry.split("|", 1)
                        if ":" in comp_info:
                            comp_name, comp_class = comp_info.split(":", 1)
                            entries.append({
                                "actor_path": actor_path,
                                "actor_proxy": RemoteObjectProxy(self, actor_path),
                                "component_name": comp_name,
                                "component_class": comp_class,
                                "component_path": f"{actor_path}.{comp_name}",
                            })
            return entries
        except UnrealRemoteError as e:
            logger.debug(f"FindActorsByComponent failed: {e}")
            return []

    def find_components(self, actor_path: str,
                        class_filter: str = "") -> list[dict]:
        """
        Find components on an actor using the RammsCoreBridge.

        Args:
            actor_path: Full object path of the actor.
            class_filter: Substring filter matched against both the component
                instance name and the class name (empty = all).

        Returns:
            List of dicts with 'name', 'class_name', and 'path' keys.
        """
        try:
            result = self._call_function(
                self.RAMMS_CORE_BRIDGE,
                "FindComponents",
                {"ActorPath": actor_path, "ClassNameFilter": class_filter}
            )
            entries = []
            if isinstance(result, dict):
                result = result.get("ReturnValue", result)
            if isinstance(result, list):
                for entry in result:
                    if isinstance(entry, str) and ":" in entry:
                        name, cls = entry.split(":", 1)
                        entries.append({
                            "name": name,
                            "class_name": cls,
                            "path": f"{actor_path}.{name}",
                        })
            return entries
        except UnrealRemoteError as e:
            logger.debug(f"RammsCoreBridge FindComponents failed: {e}")
            # Fallback to describe_object
            return self._get_components_via_describe(actor_path, class_filter)

    def _get_components_via_describe(self, actor_path: str,
                                     class_filter: str = "") -> list[dict]:
        """Fallback: get components via describe_object.

        Note: describe_object returns UPROPERTY variable names, which may
        differ from component instance names. We attempt to read the property
        value to get the actual object path.
        """
        try:
            desc = self.describe_object(actor_path)
        except UnrealRemoteError:
            return []

        components = []
        for prop in desc.get("Properties", []):
            prop_type = prop.get("Type", "")
            if not prop_type.endswith("Component"):
                continue
            prop_name = prop.get("Name", "")
            if class_filter:
                fl = class_filter.lower()
                if fl not in prop_type.lower() and fl not in prop_name.lower():
                    continue

            # Try to read the property value to get the actual component path
            comp_path = None
            try:
                val = self._get_property(actor_path, prop_name)
                if isinstance(val, str) and val:
                    comp_path = val
            except UnrealRemoteError:
                pass

            if not comp_path:
                # Best-effort: use property name (may differ from instance name)
                comp_path = f"{actor_path}.{prop_name}"

            components.append({
                "name": prop_name,
                "class_name": prop_type,
                "path": comp_path,
            })
        return components

    def _parse_path_list(self, result: Any,
                         name_filter: Optional[str] = None) -> list[RemoteObjectProxy]:
        """Parse a string array result into proxies."""
        paths = []
        if isinstance(result, list):
            paths = result
        elif isinstance(result, dict):
            # Check common response wrappings
            for key in ("ReturnValue", "result"):
                if key in result and isinstance(result[key], list):
                    paths = result[key]
                    break

        actors = []
        for entry in paths:
            path = entry if isinstance(entry, str) else str(entry)
            if not path:
                continue
            if name_filter and name_filter.lower() not in path.lower():
                continue
            actors.append(RemoteObjectProxy(self, path))
        return actors

    def _parse_actor_list(self, result: Any,
                          name_filter: Optional[str] = None) -> list[RemoteObjectProxy]:
        """Parse an actor list result from EditorActorSubsystem into proxies."""
        actors = []

        # The return value is typically a list of object path strings,
        # or a dict with a "ReturnValue" key containing the list
        paths = []
        if isinstance(result, list):
            paths = result
        elif isinstance(result, dict):
            # Could be {"ReturnValue": [...]} or other structures
            for key in ("ReturnValue", "OutActors", "result"):
                if key in result:
                    val = result[key]
                    if isinstance(val, list):
                        paths = val
                        break
            if not paths:
                # Try iterating dict values
                for val in result.values():
                    if isinstance(val, list):
                        paths = val
                        break

        for entry in paths:
            if isinstance(entry, str):
                path = entry
            elif isinstance(entry, dict):
                path = entry.get("ObjectPath", entry.get("Path", entry.get("$ObjectPath", "")))
            else:
                continue

            if not path:
                continue
            if name_filter and name_filter.lower() not in path.lower():
                continue
            actors.append(RemoteObjectProxy(self, path))

        return actors

    def search_assets(self, query: str = "",
                      class_names: Optional[list[str]] = None,
                      package_paths: Optional[list[str]] = None,
                      limit: int = 50) -> list[dict]:
        """
        Search for assets in the content browser.

        Args:
            query: Text search query
            class_names: Filter by class names (e.g. ["StaticMesh", "Material"])
            package_paths: Filter by package paths (e.g. ["/Game/MyFolder"])
            limit: Max results to return

        Returns:
            List of asset info dicts with Name, Path, Class, etc.
        """
        body: dict[str, Any] = {
            "Query": query,
            "Limit": limit,
        }
        filter_dict: dict[str, Any] = {}
        if class_names:
            filter_dict["ClassNames"] = class_names
        if package_paths:
            filter_dict["PackagePaths"] = package_paths
        if filter_dict:
            body["Filter"] = filter_dict

        try:
            result = self._request("PUT", "/remote/search/assets", body)
        except UnrealRemoteError as e:
            logger.warning(f"Asset search failed: {e}")
            return []

        if isinstance(result, dict):
            return result.get("Assets", [])
        elif isinstance(result, list):
            return result
        return []

    def describe_object(self, object_path: str) -> dict:
        """Get detailed description of an object (properties, functions, etc.)."""
        body = {"objectPath": object_path}
        return self._request("PUT", "/remote/object/describe", body)

    def get_presets(self) -> list[dict]:
        """List all Remote Control presets."""
        return self._request("GET", "/remote/presets")

    def get_preset(self, preset_name: str) -> dict:
        """Get details of a specific preset."""
        return self._request("GET", f"/remote/preset/{preset_name}")

    def get_info(self) -> dict:
        """Get Remote Control API server info and available routes."""
        return self._request("GET", "/remote/info")

    def batch(self, requests: list[dict]) -> list[Any]:
        """
        Execute multiple requests in a single batch call.

        Args:
            requests: List of request dicts, each with "RequestId", "Url",
                      "Verb", and optionally "Body".

        Returns:
            List of response dicts with "RequestId" and "ResponseBody".
        """
        body = {"Requests": requests}
        result = self._request("PUT", "/remote/batch", body)
        if isinstance(result, dict):
            return result.get("Responses", [])
        return result if isinstance(result, list) else []

    # ── Low-level operations ────────────────────────────────────────

    def _get_property(self, object_path: str, property_name: str,
                      access: str = "READ_ACCESS") -> Any:
        """Read a property from a remote object."""
        body = {
            "objectPath": object_path,
            "propertyName": property_name,
            "access": access,
        }
        result = self._request("PUT", "/remote/object/property", body)
        # Response typically has the property value nested
        if isinstance(result, dict):
            return result.get(property_name, result)
        return result

    def _set_property(self, object_path: str, property_name: str,
                      value: Any) -> Any:
        """Set a property on a remote object."""
        body = {
            "objectPath": object_path,
            "propertyName": property_name,
            "propertyValue": {property_name: value},
            "access": "WRITE_ACCESS",
        }
        return self._request("PUT", "/remote/object/property", body)

    def _call_function(self, object_path: str, function_name: str,
                       parameters: Optional[dict] = None) -> Any:
        """Call a function on a remote object."""
        body: dict[str, Any] = {
            "objectPath": object_path,
            "functionName": function_name,
        }
        if parameters:
            body["parameters"] = parameters

        result = self._request("PUT", "/remote/object/call", body)

        # Extract return value if present
        if isinstance(result, dict) and "ReturnValue" in result:
            return result["ReturnValue"]
        return result

    # ── HTTP transport ──────────────────────────────────────────────

    def _request(self, method: str, endpoint: str,
                 body: Optional[dict] = None) -> Any:
        """Make an HTTP request to the Remote Control API."""
        url = f"{self.base_url}{endpoint}"

        if body is not None:
            data = json.dumps(body).encode("utf-8")
        else:
            data = None

        _debug = logger.isEnabledFor(logging.DEBUG)
        if _debug:
            body_preview = data.decode("utf-8")[:500] if data else None
            logger.debug(f"→ {method} {endpoint} body={body_preview}")

        headers = {"Content-Type": "application/json"}
        req = Request(url, data=data, headers=headers, method=method)

        try:
            with urlopen(req, timeout=self.timeout) as resp:
                resp_data = resp.read().decode("utf-8")
                if _debug:
                    logger.debug(f"← {resp.status} {resp_data[:500] if resp_data else '(empty)'}")
                if resp_data:
                    return json.loads(resp_data)
                return {}
        except HTTPError as e:
            body_text = ""
            try:
                body_text = e.read().decode("utf-8")
            except Exception:
                pass
            if _debug:
                logger.debug(f"← HTTP {e.code}: {body_text[:500]}")
            raise UnrealRemoteError(
                f"HTTP {e.code} on {method} {endpoint}: {body_text}"
            ) from e
        except URLError as e:
            raise UnrealRemoteError(
                f"Connection failed to {url}: {e.reason}"
            ) from e
        except Exception as e:
            raise UnrealRemoteError(f"Request failed: {e}") from e

    def ping(self) -> bool:
        """Check if the Remote Control API is reachable."""
        try:
            self._request("GET", "/remote/info")
            return True
        except UnrealRemoteError:
            return False

    def __repr__(self) -> str:
        return f"UnrealRemote('{self.base_url}')"
