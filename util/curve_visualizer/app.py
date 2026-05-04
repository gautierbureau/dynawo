import streamlit as st
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import numpy as np
import io
import os
import re
import sys

from binary_curves import load_bin

st.set_page_config(page_title="Curve Visualizer", layout="wide")
st.title("Curve Visualizer")

# ── CLI preload (streamlit run app.py -- file1.csv file2.bin) ────────────────

@st.cache_data(show_spinner="Loading file…")
def _load_path_cached(path: str, mtime: float, size: int) -> pd.DataFrame:
    """Read and parse a CSV/.bin file from disk. Cached on (path, mtime, size)
    so reruns of the script don't re-read multi-GB inputs."""
    with open(path, "rb") as fh:
        raw = fh.read()
    if path.lower().endswith(".bin"):
        return load_bin(raw)
    sample = raw[:4096].decode("utf-8", errors="replace")
    first_line = sample.splitlines()[0] if sample else ""
    sep = ";" if first_line.count(";") > first_line.count(",") else ","
    return pd.read_csv(io.BytesIO(raw), sep=sep)


_preloaded: "dict[str, pd.DataFrame]" = {}
for _path in (a for a in sys.argv[1:] if not a.startswith("-")):
    if not os.path.isfile(_path):
        st.error(f"Preload path not found: {_path}")
        st.stop()
    _stat = os.stat(_path)
    _preloaded[os.path.basename(_path)] = _load_path_cached(
        _path, _stat.st_mtime, _stat.st_size
    )

# ── Palettes ───────────────────────────────────────────────────────────────────

COLORS = [
    "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728",
    "#9467bd", "#8c564b", "#e377c2", "#7f7f7f",
    "#bcbd22", "#17becf",
]

# One dash style per loaded file — same curve across files keeps its colour but
# changes dash so simulations are visually distinguishable.
DASHES = ["solid", "dash", "dot", "dashdot", "longdash", "longdashdot"]

# ── Data loading ───────────────────────────────────────────────────────────────

@st.cache_data
def load_csv(data: bytes) -> pd.DataFrame:
    # Sniff ',' vs ';' from the header line; fall back to ','.
    sample = data[:4096].decode("utf-8", errors="replace")
    first_line = sample.splitlines()[0] if sample else ""
    sep = ";" if first_line.count(";") > first_line.count(",") else ","
    return pd.read_csv(io.BytesIO(data), sep=sep)

@st.cache_data
def load_bin_cached(data: bytes) -> pd.DataFrame:
    return load_bin(data)

# ── Derived curves ─────────────────────────────────────────────────────────────

_SAFE_NAMES: dict = {
    "abs": np.abs, "sqrt": np.sqrt, "exp": np.exp,
    "log": np.log, "log2": np.log2, "log10": np.log10,
    "sin": np.sin, "cos": np.cos, "tan": np.tan,
    "arcsin": np.arcsin, "arccos": np.arccos,
    "arctan": np.arctan, "arctan2": np.arctan2,
    "sinh": np.sinh, "cosh": np.cosh, "tanh": np.tanh,
    "minimum": np.minimum, "maximum": np.maximum,
    "clip": np.clip, "sign": np.sign,
    "floor": np.floor, "ceil": np.ceil, "round": np.round,
    "where": np.where,
    "pi": np.pi, "e": np.e,
}

_BACKTICK_RE = re.compile(r"`([^`]+)`")


def _tok(col: str) -> str:
    """Return `col` if it's a valid Python identifier, else wrap in backticks."""
    return col if col.isidentifier() else f"`{col}`"


def _on_add_derived(label: str, expr_key: str, existing_cols: set[str]) -> None:
    """Callback for the guided builder's "Add" button.

    Runs before widgets are re-instantiated on the resulting rerun, so it may
    mutate ``st.session_state[expr_key]`` (the text area feeding `apply_derived`).
    """
    err_key = f"derived_form_err__{label}"
    op = st.session_state.get(f"derived_op__{label}")
    new_name = st.session_state.get(f"derived_name__{label}", "").strip()

    err: str | None = None
    expr: str | None = None

    if not new_name:
        err = "Give the new curve a name."
    elif not new_name.isidentifier():
        err = f"Invalid name `{new_name}` (must be a plain identifier)."
    elif new_name in existing_cols:
        err = f"`{new_name}` is already a defined curve."
    elif op == "sum":
        sel = st.session_state.get(f"derived_sum__{label}", []) or []
        if len(sel) < 2:
            err = "Pick at least two curves to sum."
        else:
            expr = " + ".join(_tok(c) for c in sel)
    elif op == "product":
        sel = st.session_state.get(f"derived_prod__{label}", []) or []
        if len(sel) < 2:
            err = "Pick at least two curves to multiply."
        else:
            expr = " * ".join(_tok(c) for c in sel)
    elif op == "scale":
        curve  = st.session_state.get(f"derived_scale_curve__{label}")
        factor = st.session_state.get(f"derived_scale_factor__{label}", 1.0)
        if curve is None:
            err = "Pick a curve to scale."
        else:
            expr = f"{factor:g} * {_tok(curve)}"
    elif op == "modulus":
        re_c = st.session_state.get(f"derived_mod_re__{label}")
        im_c = st.session_state.get(f"derived_mod_im__{label}")
        if re_c is None or im_c is None:
            err = "Pick both real and imaginary parts."
        else:
            expr = f"sqrt({_tok(re_c)}**2 + {_tok(im_c)}**2)"
    elif op == "log":
        curve = st.session_state.get(f"derived_log_curve__{label}")
        base  = st.session_state.get(f"derived_log_base__{label}", "e")
        if curve is None:
            err = "Pick a curve."
        else:
            fn = {"e": "log", "10": "log10", "2": "log2"}[base]
            expr = f"{fn}({_tok(curve)})"

    if err or expr is None:
        st.session_state[err_key] = err or "Unknown error"
        return

    st.session_state[err_key] = ""
    cur = st.session_state.get(expr_key, "")
    sep = "" if (not cur or cur.endswith("\n")) else "\n"
    st.session_state[expr_key] = cur + sep + f"{new_name} = {expr}"


def apply_derived(df: pd.DataFrame, spec: str) -> tuple[pd.DataFrame, list[str]]:
    """Parse `name = expr` lines; append derived columns. Returns (df, errors).

    A later line may reference a curve defined by an earlier successful line.
    Duplicate definitions and attempts to override a source column are rejected.
    """
    errors: list[str] = []
    if not spec.strip():
        return df, errors
    df = df.copy()
    source_cols = set(df.columns)
    defined: set[str] = set()
    time_col = df.columns[0]
    for lineno, raw_line in enumerate(spec.splitlines(), 1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            errors.append(f"line {lineno}: missing '=' in `{raw_line}`")
            continue
        name, expr = line.split("=", 1)
        name, expr = name.strip(), expr.strip()
        if not name.isidentifier():
            errors.append(f"line {lineno}: invalid curve name `{name}`")
            continue
        if name == time_col:
            errors.append(f"line {lineno}: cannot override time column `{name}`")
            continue
        if name in source_cols:
            errors.append(f"line {lineno}: cannot override source column `{name}`")
            continue
        if name in defined:
            errors.append(f"line {lineno}: `{name}` is already defined earlier")
            continue
        placeholders: dict[str, str] = {}
        def _sub(m: re.Match) -> str:
            key = f"__col_{len(placeholders)}"
            placeholders[key] = m.group(1)
            return key
        rewritten = _BACKTICK_RE.sub(_sub, expr)
        missing = [c for c in placeholders.values() if c not in df.columns]
        if missing:
            errors.append(f"line {lineno}: unknown column(s) {missing}")
            continue
        ns: dict = dict(_SAFE_NAMES)
        for c in df.columns:
            if c.isidentifier():
                ns[c] = df[c]
        for key, col in placeholders.items():
            ns[key] = df[col]
        try:
            df[name] = eval(rewritten, {"__builtins__": {}}, ns)
        except Exception as exc:
            errors.append(f"line {lineno} (`{name}`): {exc}")
            continue
        defined.add(name)
    return df, errors


@st.cache_data
def generate_demo(seed: int = 0) -> pd.DataFrame:
    rng = np.random.default_rng(seed)
    t = np.linspace(0, 10, 500)
    phase = rng.uniform(0, 0.5)
    scale = rng.uniform(0.8, 1.2)
    return pd.DataFrame({
        "time":        t,
        "sine":        scale * np.sin(2 * np.pi * t / 5 + phase),
        "cosine":      scale * np.cos(2 * np.pi * t / 5 + phase),
        "exp_decay":   np.exp(-t / (3 + rng.uniform(-0.5, 0.5))),
        "linear":      (0.1 + rng.uniform(-0.02, 0.02)) * t - 0.5,
        "large_scale": 1000 * scale * np.sin(2 * np.pi * t / 8 + 1 + phase),
    })

uploaded_files = st.file_uploader(
    "Upload one or more CSV or binary (.bin) files "
    "(first column = time/x axis, remaining = curves)",
    type=["csv", "bin"],
    accept_multiple_files=True,
)

files: dict[str, pd.DataFrame] = dict(_preloaded)
if uploaded_files:
    for f in uploaded_files:
        raw = f.read()
        if f.name.endswith(".bin"):
            files[f.name] = load_bin_cached(raw)
        else:
            files[f.name] = load_csv(raw)

if not files:
    st.info("Please upload one or more CSV or binary (.bin) files to visualize.")
    st.stop()

file_labels = list(files.keys())

# ── Derived curves UI ──────────────────────────────────────────────────────────

with st.expander("Derived curves", expanded=False):
    st.caption(
        "Define new curves as `name = expression`, one per line. "
        "Wrap column names containing dots in backticks (e.g. `` `resistor.R` ``). "
        "Available: `sqrt`, `exp`, `log`/`log2`/`log10`, `sin`/`cos`/`tan`, "
        "`arctan2`, `abs`, `sign`, `where`, `minimum`/`maximum`, `clip`, `pi`, `e`."
    )
    if len(file_labels) > 1:
        containers = st.tabs(file_labels)
    else:
        containers = [st.container()]
    for container, label in zip(containers, file_labels):
        with container:
            expr_key  = f"derived__{label}"
            pick_key  = f"derived_pick__{label}"
            ins_key   = f"derived_ins__{label}"

            # Evaluate the current spec up front so derived curves become
            # selectable in the Insert picker and in the guided builder below.
            spec_current = st.session_state.get(expr_key, "")
            working_df, derived_errs = apply_derived(files[label], spec_current)
            files[label] = working_df
            curve_cols    = list(working_df.columns[1:])
            existing_cols = set(working_df.columns)

            pick_col, btn_col = st.columns([4, 1])
            with pick_col:
                picked = st.selectbox(
                    "Column to insert",
                    options=curve_cols,
                    key=pick_key,
                    index=None,
                    placeholder="Pick a column (type to filter)…",
                    label_visibility="collapsed",
                )
            with btn_col:
                if st.button(
                    "Insert",
                    key=ins_key,
                    use_container_width=True,
                    disabled=picked is None,
                ):
                    token = _tok(picked)
                    cur = st.session_state.get(expr_key, "")
                    sep = "" if (not cur or cur[-1] in " \t\n(=+-*/,") else " "
                    st.session_state[expr_key] = cur + sep + token

            st.text_area(
                f"Expressions for {label}",
                key=expr_key,
                height=120,
                placeholder="magnitude = sqrt(re**2 + im**2)\ntotal = sine + cosine\nscaled = 2.5 * sine",
                label_visibility="collapsed",
            )
            for err in derived_errs:
                st.error(err)

            # --- Guided builder ------------------------------------------------
            st.divider()
            st.markdown("**Guided builder**")

            op_col, name_col = st.columns([2, 3])
            with op_col:
                op = st.selectbox(
                    "Operation",
                    options=["sum", "product", "scale", "modulus", "log"],
                    key=f"derived_op__{label}",
                )
            with name_col:
                new_name = st.text_input(
                    "New curve name",
                    key=f"derived_name__{label}",
                    placeholder="e.g. magnitude",
                )

            sum_sel = prod_sel = None
            scale_curve = log_curve = None
            factor = None
            re_c = im_c = None
            base = None

            if op == "sum":
                sum_sel = st.multiselect(
                    "Curves to sum",
                    options=curve_cols,
                    key=f"derived_sum__{label}",
                )
            elif op == "product":
                prod_sel = st.multiselect(
                    "Curves to multiply",
                    options=curve_cols,
                    key=f"derived_prod__{label}",
                )
            elif op == "scale":
                c1, c2 = st.columns([3, 2])
                with c1:
                    scale_curve = st.selectbox(
                        "Curve",
                        options=curve_cols,
                        key=f"derived_scale_curve__{label}",
                        index=None,
                        placeholder="Pick a curve…",
                    )
                with c2:
                    factor = st.number_input(
                        "Factor",
                        value=1.0,
                        key=f"derived_scale_factor__{label}",
                        format="%g",
                    )
            elif op == "modulus":
                c1, c2 = st.columns(2)
                with c1:
                    re_c = st.selectbox(
                        "Real part",
                        options=curve_cols,
                        key=f"derived_mod_re__{label}",
                        index=None,
                        placeholder="Real…",
                    )
                with c2:
                    im_c = st.selectbox(
                        "Imaginary part",
                        options=curve_cols,
                        key=f"derived_mod_im__{label}",
                        index=None,
                        placeholder="Imaginary…",
                    )
            elif op == "log":
                c1, c2 = st.columns([3, 2])
                with c1:
                    log_curve = st.selectbox(
                        "Curve",
                        options=curve_cols,
                        key=f"derived_log_curve__{label}",
                        index=None,
                        placeholder="Pick a curve…",
                    )
                with c2:
                    base = st.selectbox(
                        "Base",
                        options=["e", "10", "2"],
                        key=f"derived_log_base__{label}",
                    )

            st.button(
                "Add to expressions",
                key=f"derived_add_form__{label}",
                type="primary",
                on_click=_on_add_derived,
                args=(label, expr_key, existing_cols),
            )
            form_err = st.session_state.get(f"derived_form_err__{label}", "")
            if form_err:
                st.warning(form_err)

# Collect all unique curve columns across all files (preserving first-seen order)
all_curve_cols: list[str] = []
_seen: set[str] = set()
for df in files.values():
    for col in list(df.columns[1:]):
        if col not in _seen:
            all_curve_cols.append(col)
            _seen.add(col)

# Stable colour per curve name, stable dash per file
col_color  = {col: COLORS[i % len(COLORS)] for i, col in enumerate(all_curve_cols)}
file_dash  = {lbl: DASHES[i % len(DASHES)]  for i, lbl in enumerate(file_labels)}

# When there are many curves, default everything to hidden so the app does not
# try to render hundreds of traces on load.  Users can then selectively enable
# the curves they care about.
MANY_CURVES_THRESHOLD = 10
many_curves = len(all_curve_cols) > MANY_CURVES_THRESHOLD

def safe_key(*parts: str) -> str:
    """Build a widget key that is safe regardless of file/column names."""
    return "__".join(str(p) for p in parts)


# -- Multiselect state workaround ------------------------------------------------
# Streamlit sometimes drops a multiselect's session_state when the `options` list
# changes between reruns (e.g. a derived curve is added via Ctrl+Enter in the
# expressions text area).  We mirror each relevant multiselect's value into a
# separate slot keyed ``managed__<widget_key>`` via an on_change callback, and
# restore it back to the widget's key just before the widget instantiates.

def _save_multiselect_to_managed(widget_key: str) -> None:
    """on_change callback: copy the widget's current value into its managed slot."""
    st.session_state[f"managed__{widget_key}"] = list(
        st.session_state.get(widget_key, [])
    )


def _restore_multiselect(widget_key: str, valid_options: list[str]) -> None:
    """Re-seed ``session_state[widget_key]`` from its managed slot.

    Filters the saved selection to only items present in ``valid_options``.
    Must be called *before* the multiselect with ``key=widget_key`` renders.
    """
    managed_key = f"managed__{widget_key}"
    saved = [x for x in st.session_state.get(managed_key, []) if x in valid_options]
    st.session_state[managed_key] = saved
    st.session_state[widget_key]  = saved

# ── Tabs ───────────────────────────────────────────────────────────────────────

tab_overlay, tab_grid = st.tabs(["Overlay", "Grid"])

# ══════════════════════════════════════════════════════════════════════════════
# TAB 1 – OVERLAY
# ══════════════════════════════════════════════════════════════════════════════

with tab_overlay:
    ctrl_col, plot_col = st.columns([1, 3])

    # settings[(file_label, col)] = {enabled, axis}
    ov_settings: dict[tuple[str, str], dict] = {}

    with ctrl_col:
        if many_curves:
            st.caption(
                f"{len(all_curve_cols)} curves — select variables to plot using the pickers below."
            )
        for fi, (label, df) in enumerate(files.items()):
            curve_cols = list(df.columns[1:])
            dash_name  = file_dash[label]
            # Small dash-style preview next to simulation name
            dash_css = {
                "solid":       "4px solid",
                "dash":        "4px dashed",
                "dot":         "4px dotted",
                "dashdot":     "4px dashed",   # approximation in CSS
                "longdash":    "4px dashed",
                "longdashdot": "4px dashed",
            }[dash_name]
            with st.expander(f"**{label}**", expanded=not many_curves):
                if many_curves:
                    ov_left_key  = safe_key("ov_left", fi)
                    ov_right_key = safe_key("ov_right", fi)
                    _restore_multiselect(ov_left_key,  curve_cols)
                    _restore_multiselect(ov_right_key, curve_cols)
                    left_sel = st.multiselect(
                        "Left axis",
                        options=curve_cols,
                        key=ov_left_key,
                        on_change=_save_multiselect_to_managed,
                        args=(ov_left_key,),
                    )
                    right_sel = st.multiselect(
                        "Right axis",
                        options=curve_cols,
                        key=ov_right_key,
                        on_change=_save_multiselect_to_managed,
                        args=(ov_right_key,),
                    )
                    for col in curve_cols:
                        in_left  = col in left_sel
                        in_right = col in right_sel
                        ov_settings[(label, col)] = {
                            "enabled": in_left or in_right,
                            # right_sel takes priority if listed in both
                            "axis": "Right" if in_right else "Left",
                        }
                else:
                    for col in curve_cols:
                        color = col_color[col]
                        st.markdown(
                            f'<span style="display:inline-block;width:20px;height:3px;'
                            f'border-top:{dash_css} {color};vertical-align:middle;'
                            f'margin-right:6px"></span>**{col}**',
                            unsafe_allow_html=True,
                        )
                        c1, c2 = st.columns([1, 2])
                        with c1:
                            enabled = st.checkbox(
                                "Show", value=True,
                                key=safe_key("ov_show", fi, col),
                            )
                        with c2:
                            axis = st.radio(
                                "Y axis",
                                options=["Left", "Right"],
                                horizontal=True,
                                key=safe_key("ov_axis", fi, col),
                                label_visibility="collapsed",
                            )
                        ov_settings[(label, col)] = {"enabled": enabled, "axis": axis}

    with plot_col:
        has_left = any(
            cfg["axis"] == "Left"
            for (_, col), cfg in ov_settings.items()
            if cfg["enabled"]
        )
        has_right = any(
            cfg["axis"] == "Right"
            for (_, col), cfg in ov_settings.items()
            if cfg["enabled"]
        )

        fig = make_subplots(specs=[[{"secondary_y": True}]])

        for (label, col), cfg in ov_settings.items():
            if not cfg["enabled"]:
                continue
            df = files[label]
            if col not in df.columns:
                continue
            x_col = df.columns[0]
            fig.add_trace(
                go.Scatter(
                    x=df[x_col], y=df[col],
                    name=f"{col} [{label}]",
                    legendgroup=col,
                    line=dict(color=col_color[col], dash=file_dash[label]),
                    mode="lines",
                ),
                secondary_y=(cfg["axis"] == "Right"),
            )

        # Determine a sensible x-axis label (use first file's x column name)
        x_title = list(files.values())[0].columns[0]
        fig.update_layout(
            height=600,
            hovermode="x unified",
            showlegend=True,
            legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
            margin=dict(l=60, r=60, t=40, b=60),
            xaxis=dict(title=x_title),
        )
        fig.update_yaxes(title_text="Left axis",  secondary_y=False, autorange=True,
                         showgrid=(has_left or not has_right))
        fig.update_yaxes(title_text="Right axis", secondary_y=True,  autorange=True,
                         showgrid=(not has_left and has_right), visible=has_right)

        st.plotly_chart(fig, use_container_width=True)

        # Dash-style legend for simulations
        legend_html = " &nbsp;&nbsp; ".join(
            f'<span style="border-top:3px dashed {col_color.get(list(files[lbl].columns[1:2])[0], "#888") if list(files[lbl].columns[1:2]) else "#888"};'
            f'display:inline-block;width:24px;height:0;vertical-align:middle;margin-right:4px;'
            f'border-style:{file_dash[lbl].replace("longdash","dashed").replace("dashdot","dashed").replace("dot","dotted").replace("dash","dashed").replace("solid","solid")}"></span>{lbl}'
            for lbl in file_labels
        )
        st.caption(
            "Line style per simulation: &nbsp;"
            + " &nbsp;|&nbsp; ".join(
                f'<code style="border-bottom:2px {file_dash[lbl].replace("longdash","dashed").replace("dashdot","dashed").replace("dot","dotted").replace("dash","dashed")} #555">{lbl}</code>'
                for lbl in file_labels
            ),
            unsafe_allow_html=True,
        )

# ══════════════════════════════════════════════════════════════════════════════
# TAB 2 – GRID
# ══════════════════════════════════════════════════════════════════════════════

with tab_grid:
    LAYOUTS = {
        "1 row × 2 cols":  (1, 2),
        "2 rows × 1 col":  (2, 1),
        "2 rows × 2 cols": (2, 2),
        "3 rows × 1 col":  (3, 1),
        "1 row × 3 cols":  (1, 3),
    }

    top1, top2 = st.columns([2, 2])
    with top1:
        layout_choice = st.selectbox("Layout", options=list(LAYOUTS.keys()), index=2, key="grid_layout")
    with top2:
        shared_x = st.checkbox("Shared x axis (synchronised zoom)", value=True, key="grid_shared_x")

    n_rows, n_cols = LAYOUTS[layout_choice]
    n_panels  = n_rows * n_cols
    panel_labels = [f"Panel {i + 1}" for i in range(n_panels)]

    st.divider()

    ctrl_col, plot_col = st.columns([1, 3])

    gr_settings: dict[tuple[str, str], dict] = {}

    with ctrl_col:
        if many_curves:
            st.caption(
                f"{len(all_curve_cols)} curves — use the pickers below to assign variables to panels."
            )
        curve_file_idx = 0   # global index for default panel distribution
        for fi, (label, df) in enumerate(files.items()):
            curve_cols = list(df.columns[1:])
            with st.expander(f"**{label}**", expanded=not many_curves):
                if many_curves:
                    # One multiselect per panel (left axis) replaces n_vars × n_panels widgets.
                    panel_assignments: dict[str, tuple[int, str]] = {}
                    for pi, plabel in enumerate(panel_labels):
                        gr_left_key  = safe_key("gr_left",  fi, pi)
                        gr_right_key = safe_key("gr_right", fi, pi)
                        _restore_multiselect(gr_left_key,  curve_cols)
                        _restore_multiselect(gr_right_key, curve_cols)
                        sel_l = st.multiselect(
                            f"{plabel} – Left",
                            options=curve_cols,
                            key=gr_left_key,
                            on_change=_save_multiselect_to_managed,
                            args=(gr_left_key,),
                        )
                        sel_r = st.multiselect(
                            f"{plabel} – Right",
                            options=curve_cols,
                            key=gr_right_key,
                            on_change=_save_multiselect_to_managed,
                            args=(gr_right_key,),
                        )
                        for col in sel_l:
                            panel_assignments[col] = (pi, "Left")
                        for col in sel_r:
                            panel_assignments[col] = (pi, "Right")
                    for col in curve_cols:
                        assignment = panel_assignments.get(col)
                        gr_settings[(label, col)] = {
                            "panel": assignment[0] if assignment else None,
                            "axis":  assignment[1] if assignment else "Left",
                        }
                else:
                    for col in curve_cols:
                        color = col_color[col]
                        st.markdown(
                            f'<span style="display:inline-block;width:12px;height:12px;'
                            f'border-radius:50%;background:{color};margin-right:6px"></span>'
                            f'**{col}**',
                            unsafe_allow_html=True,
                        )
                        c1, c2 = st.columns([3, 2])
                        with c1:
                            panel = st.selectbox(
                                "Panel",
                                options=["—"] + panel_labels,
                                index=min(curve_file_idx % n_panels + 1, n_panels),
                                key=safe_key("gr_panel", fi, col),
                                label_visibility="collapsed",
                            )
                        with c2:
                            axis = st.radio(
                                "Y",
                                options=["L", "R"],
                                horizontal=True,
                                key=safe_key("gr_axis", fi, col),
                                label_visibility="collapsed",
                            )
                        gr_settings[(label, col)] = {
                            "panel": None if panel == "—" else panel_labels.index(panel),
                            "axis":  "Left" if axis == "L" else "Right",
                        }
                        curve_file_idx += 1

    with plot_col:
        specs = [[{"secondary_y": True} for _ in range(n_cols)] for _ in range(n_rows)]

        fig2 = make_subplots(
            rows=n_rows, cols=n_cols,
            specs=specs,
            shared_xaxes=shared_x,
            subplot_titles=panel_labels if n_panels > 1 else None,
            horizontal_spacing=0.08,
            vertical_spacing=0.12,
        )

        panels_with_right: set[int] = set()
        panels_with_left:  set[int] = set()

        for (label, col), cfg in gr_settings.items():
            panel_idx = cfg["panel"]
            if panel_idx is None:
                continue
            df = files[label]
            if col not in df.columns:
                continue
            x_col = df.columns[0]
            row_pos = panel_idx // n_cols + 1
            col_pos = panel_idx %  n_cols + 1
            secondary = cfg["axis"] == "Right"
            if secondary:
                panels_with_right.add(panel_idx)
            else:
                panels_with_left.add(panel_idx)

            fig2.add_trace(
                go.Scatter(
                    x=df[x_col], y=df[col],
                    name=f"{col} [{label}]",
                    legendgroup=f"{col}__{label}",
                    line=dict(color=col_color[col], dash=file_dash[label]),
                    mode="lines",
                ),
                row=row_pos, col=col_pos,
                secondary_y=secondary,
            )

        x_title = list(files.values())[0].columns[0]
        for c in range(1, n_cols + 1):
            key_n = (n_rows - 1) * n_cols + c
            axis_key = "xaxis" if key_n == 1 else f"xaxis{key_n}"
            fig2.update_layout(**{axis_key: dict(title=x_title)})

        fig2.update_layout(
            height=280 * n_rows + 60,
            hovermode="x unified",
            showlegend=True,
            legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
            margin=dict(l=60, r=60, t=60, b=60),
        )

        for panel_idx in range(n_panels):
            row_pos = panel_idx // n_cols + 1
            col_pos = panel_idx %  n_cols + 1
            p_has_left  = (panel_idx in panels_with_left)
            p_has_right = (panel_idx in panels_with_right)
            fig2.update_yaxes(autorange=True,
                              showgrid=(p_has_left or not p_has_right),
                              row=row_pos, col=col_pos, secondary_y=False)
            fig2.update_yaxes(autorange=True,
                              showgrid=(not p_has_left and p_has_right),
                              visible=p_has_right,
                              row=row_pos, col=col_pos, secondary_y=True)

        st.plotly_chart(fig2, use_container_width=True)

# ── Raw data preview ───────────────────────────────────────────────────────────

with st.expander("Raw data"):
    raw_tabs = st.tabs(file_labels)
    for tab, (label, df) in zip(raw_tabs, files.items()):
        with tab:
            st.dataframe(df, use_container_width=True)
