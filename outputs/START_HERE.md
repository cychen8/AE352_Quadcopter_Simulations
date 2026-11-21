# Reading the simulation results

Open `figures/all_mission_figures.pdf` to browse all figures. Individual vector
PDFs can be uploaded to Overleaf. Figures use integrated states and applied motor inputs.

- `RESULTS.md`: current mission energies, measured performance, and pass/fail checks.
- `metrics.json`: full numerical results with the chosen engineering tolerances.
- `parameters_used.json`: physical parameters and controller settings used for the run.
- `validation.json`: independent vector-form checks and time-step sensitivity results.
- `report_figures.tex`: figure inclusion code and simulation-method description.

Run `python run_missions.py` followed by `python validate.py` after changing parameters.
The validation halves both integration step and controller update period.

These are predictions for selected hardware under the documented model assumptions.
No wind or sensor noise is present; ground contact is ideal. Battery voltage sag,
motor electrical dynamics and thermal limits are not simulated. See the main README
for the parameter basis and limitations. Passing numerical checks does not establish
real-flight performance.
