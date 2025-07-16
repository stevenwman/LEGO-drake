import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import webbrowser
import os

# --- Configuration ---
# 1. Path to your data file
CSV_FILE_PATH = 'run_sim_save_data/data_for_2025-07-16_16-52-39/all_data_2025-07-16_16-52-39.csv'

# 2. Name of the column for the x-axis (e.g., 'time').
#    Set to None to use the row number (index).
TIME_COLUMN = 'time'

# 3. How many stacked plot windows you want.
NUM_PLOT_WINDOWS = 5
# --- End Configuration ---

# Load data
try:
    df = pd.read_csv(CSV_FILE_PATH)
    print(f"Successfully loaded {CSV_FILE_PATH}")
except FileNotFoundError:
    print(f"Error: The file '{CSV_FILE_PATH}' was not found. Exiting.")
    exit()

# Prepare data for plotting
data_columns = [col for col in df.columns if col != TIME_COLUMN]
x_axis_data = df[TIME_COLUMN] if TIME_COLUMN and TIME_COLUMN in df.columns else df.index
x_axis_title = TIME_COLUMN if TIME_COLUMN and TIME_COLUMN in df.columns else "Timestep (Index)"

# Create stacked subplots with a shared x-axis
fig = make_subplots(
    rows=NUM_PLOT_WINDOWS,
    cols=1,
    shared_xaxes=True,
    vertical_spacing=0.05
)

# --- Create dropdown menus for each plot window ---
updatemenus = []
for i in range(NUM_PLOT_WINDOWS):
    # Add an initial, visible trace to each subplot
    initial_trace_name = data_columns[i % len(data_columns)] # Pick a default column
    fig.add_trace(go.Scatter(
        x=x_axis_data,
        y=df[initial_trace_name],
        name=initial_trace_name,
        mode='lines'
    ), row=i + 1, col=1)

    # Create the buttons for the dropdown menu
    buttons = []
    for col_name in data_columns:
        buttons.append(dict(
            method='restyle',
            label=col_name,
            args=[
                {'y': [df[col_name]], 'name': [col_name]}, # New data for the trace
                [i] # Index of the trace to update (one trace per subplot)
            ]
        ))

    # Create the dropdown menu for this specific subplot
    updatemenus.append(dict(
        buttons=buttons,
        direction='down',
        showactive=True,
        # Position the dropdown menu above its corresponding plot
        x=0.0,
        xanchor='left',
        y=1 - (i * (1.0 / NUM_PLOT_WINDOWS)) + (0.15/NUM_PLOT_WINDOWS), # Vertical positioning
        yanchor='top'
    ))


# Update the layout with the dropdowns and final touches
fig.update_layout(
    height=250 * NUM_PLOT_WINDOWS + 50,
    updatemenus=updatemenus,
    title_text="Synced Plots with Independent Series Selection",
    showlegend=False # The dropdowns replace the need for a legend
)
fig.update_xaxes(title_text=x_axis_title, row=NUM_PLOT_WINDOWS, col=1)


# Save to an HTML file and open in the browser
output_filename = "interactive_dropdown_plot.html"
fig.write_html(output_filename)
webbrowser.open('file://' + os.path.realpath(output_filename))

print(f"\nPlot has been saved to {output_filename} and opened in your browser.")