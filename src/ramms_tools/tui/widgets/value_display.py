"""Value display widget — shows a labeled 3D vector or scalar."""

from __future__ import annotations

from textual.app import ComposeResult
from textual.containers import Vertical
from textual.widget import Widget
from textual.widgets import Static


class ValueDisplay(Widget):
    """Displays a labeled set of named numeric values (e.g. a 3D vector).

    Example layout::

        Position (cm)
          X:  123.45
          Y:  456.78
          Z:  789.01
    """

    DEFAULT_CSS = """
    ValueDisplay {
        width: 1fr;
        height: auto;
        padding: 0 1;
        margin: 0 1 1 0;
        border: round $primary;
    }
    ValueDisplay .vd-title {
        text-style: bold;
        color: $accent;
        padding: 0 0 0 0;
    }
    ValueDisplay .vd-row {
        padding: 0 0 0 1;
    }
    """

    def __init__(
        self,
        title: str,
        keys: tuple[str, ...] = ("x", "y", "z"),
        unit: str = "",
        precision: int = 2,
        **kwargs,
    ) -> None:
        super().__init__(**kwargs)
        self._title = title
        self._keys = keys
        self._unit = unit
        self._precision = precision
        self._widget_prefix = title.lower().replace(" ", "_").replace("(", "").replace(")", "")

    def compose(self) -> ComposeResult:
        label = self._title
        if self._unit:
            label += f" ({self._unit})"
        yield Static(label, classes="vd-title")
        for key in self._keys:
            yield Static(
                f"  {key.upper():>5s}: {'0':>10s}",
                classes="vd-row",
                id=f"vd-{self._widget_prefix}-{key}",
            )

    def update_values(self, values: dict) -> None:
        """Update displayed values from a dict like {'x': 1.0, 'y': 2.0}."""
        fmt = f"{{:.{self._precision}f}}"
        for key in self._keys:
            val = values.get(key, 0.0)
            try:
                w = self.query_one(f"#vd-{self._widget_prefix}-{key}", Static)
                w.update(f"  {key.upper():>5s}: {fmt.format(val):>10s}")
            except Exception:
                pass
