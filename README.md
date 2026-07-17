# Follow-the-Leader Project Page

This folder contains a static academic project page with two connected browser demos:

1. an interactive 3D line–arc / biarc shape planner converted from the supplied MATLAB `opt_arc_modes` prototype; and
2. a two-segment, three-stage follow-the-leader simulation converted from the supplied constant-curvature forward model and rod-length trajectory-planning logic.

## Files

```text
index.html          Academic project page and interface
styles.css          Page, planner, and simulation styles
shape-planner.js    Interactive J/C/S reference-shape planner
ftl-simulation.js   Two-segment forward model, three-stage IK planner, and playback
README.md           Run, deployment, and customization notes
```

## Shape-planning demo

- J line–arc mode
- Coplanar and spatial C biarcs
- Coplanar and spatial S biarcs
- Drag-based editing of P1 and P2
- Terminal-tangent yaw and pitch controls
- Primitive-length bounds
- Finite-difference Jacobian
- Levenberg–Marquardt damping and backtracking line search
- Tangent-continuity and length-bound residuals
- Geometric FTL path replay
- Trajectory CSV export and PNG capture

## Three-stage robot simulation

The simulation below the Method section automatically uses the current reference shape from the planner.

- Two constant-curvature robot segments
- Six absolute rod lengths, `L1`–`L6`
- Rod bounds and per-step rod-change constraints
- Stage 1: optimize all rods so `p02` follows arc 1
- Stage 2: make `p01` finish arc 1 while `p02` enters arc 2
- Stage 3: hold `L1`–`L3` fixed and optimize `L4`–`L6` to finish arc 2
- Browser-side bounded damped least-squares inverse kinematics
- Playback slider and speed control
- Base, middle, and tip coordinate frames
- Optional side projections
- Tip- and shape-deviation summaries
- CSV export containing the full `L` plan and derived `q` commands

The browser optimizer is a dependency-free numerical reproduction of the MATLAB workflow. It does not call MATLAB or require a backend server.

## Run locally

Preview the page through a local web server rather than double-clicking `index.html`:

```bash
cd ftl_project_page
python -m http.server 8000
```

Open:

```text
http://localhost:8000
```

The Three.js modules are loaded from jsDelivr, so the current package requires an internet connection while the page loads.

## Publish with GitHub Pages

1. Create a GitHub repository, for example `follow-the-leader`.
2. Copy every file from this folder into the repository root.
3. Commit and push to the `main` branch.
4. Open **Settings → Pages**.
5. Select **Deploy from a branch**, `main`, and `/(root)`.
6. Open `https://YOUR_USERNAME.github.io/follow-the-leader/` after the deployment completes.

The repository root must look like this:

```text
follow-the-leader/
├── index.html
├── styles.css
├── shape-planner.js
├── ftl-simulation.js
└── README.md
```

## Customize the paper page

In `index.html`, replace the project title, author information, affiliation, paper/code URLs, paper-specific method text, mechanism and experiment placeholders, and BibTeX.

Add project media using relative paths, for example:

```text
static/images/
static/videos/
static/pdf/
```

```html
<img src="./static/images/system_overview.png" alt="System overview">
```

## Validation note

The simulation uses the equations and stage structure supplied with the MATLAB prototypes, but the constrained optimizer is implemented in JavaScript rather than MATLAB `fmincon`. Validate exported representative trajectories against the MATLAB implementation before reporting numerical results in a paper. Hardware dynamics, collision checking, sensing noise, and closed-loop feedback are not included in this visualization.

## Display tuning in this version

- The FTL simulation uses the same 1440 px maximum page width and 720 px viewer height as the shape planner.
- Playback defaults to 0.75× with a slower 8-frame/s base rate.
- The two robot sections are rendered as real Three.js tubes, so their thickness is visible consistently across browsers.
