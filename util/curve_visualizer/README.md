# Curve Visualizer

Interactive Streamlit app for plotting Dynawo curves, plus a command-line
utility for working with binary (`.bin`) curves files.

## Features

- Upload any CSV file (first column = x/time axis, remaining columns = curves)
- Upload any Dynawo `.bin` file produced by the `BINARY` curves export mode
- Toggle each curve on/off with a checkbox
- Assign each curve to the **left** or **right** y-axis (for different magnitude scales)
- Automatic colors
- Interactive zoom, pan, and hover via Plotly
- Falls back to built-in demo data when no file is uploaded

## Usage

```bash
pip install -r requirements.txt
streamlit run app.py --server.maxUploadSize 1024
```

## CSV format

```
time,voltage,current,temperature,power
0.0,230.1,1.23,25.0,283.0
0.1,229.8,1.25,25.1,287.3
...
```

## `binary_curves.py` command-line tool

`binary_curves.py` is a companion script that ships alongside the app. It
parses the same file format the Streamlit app reads, but is usable on its
own — useful for inspecting or slicing very large `.bin` files (tens of GB)
without opening them in the browser.

### List every variable name

```bash
python binary_curves.py names simulation.bin
# → writes simulation.names.txt with one variable name per line

python binary_curves.py names simulation.bin -o vars.txt
```

Only the header is read, so this is fast even on multi-GB files.

### Extract a subset of columns as CSV

```bash
# Glob-style patterns. Multiple --pattern flags are OR-combined.
python binary_curves.py extract simulation.bin -p '*voltage*' -p 'bus?_V'

# Plain substring filter (-c, repeatable, OR-combined with -p)
python binary_curves.py extract simulation.bin -c generator
python binary_curves.py extract simulation.bin -c generator -c load

# Regular expressions (applies to --pattern only)
python binary_curves.py extract simulation.bin --regex -p '.*theta.*'

# Custom output path and number format
python binary_curves.py extract simulation.bin -p '*' -o all.csv --fmt '%.12g'

# Smaller streaming chunks for tight-memory environments
python binary_curves.py extract simulation.bin -c generator --chunk-bytes 67108864
```

The CSV always starts with a `time` column followed by the matching variables
in the order they appear in the file. Records are read chunk-by-chunk
(`--chunk-bytes`, default 1 GiB), so files that do not fit in RAM are
processed without issue.

### Programmatic use

```python
from binary_curves import load_bin, read_header, extract_to_csv

# Whole-file load (suits in-memory use, returns a DataFrame):
with open("simulation.bin", "rb") as f:
    df = load_bin(f.read())

# Header-only inspection (cheap on huge files):
with open("simulation.bin", "rb") as f:
    header = read_header(f)
print(header.n_vars, header.names[:5])

# Streaming extract:
extract_to_csv("simulation.bin", "subset.csv", ["*voltage*"])
```
