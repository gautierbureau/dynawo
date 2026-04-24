# Curve Visualizer

Interactive Streamlit app for plotting multiple curves with dual y-axis support.

## Features

- Upload any CSV file (first column = x/time axis, remaining columns = curves)
- Toggle each curve on/off with a checkbox
- Assign each curve to the **left** or **right** y-axis (for different magnitude scales)
- Automatic colors
- Interactive zoom, pan, and hover via Plotly
- Falls back to built-in demo data when no file is uploaded

## Usage

```bash
pip install -r requirements.txt
streamlit run app.py --server.maxUploadSize 900
```

## CSV format

```
time,voltage,current,temperature,power
0.0,230.1,1.23,25.0,283.0
0.1,229.8,1.25,25.1,287.3
...
```
