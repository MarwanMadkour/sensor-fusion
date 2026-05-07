# sensor-fusion

## What the project is

This is a **multi-sensor radar target tracking simulator** written in MATLAB. It simulates the full lifecycle of tracking moving objects (like aircraft or vehicles) using two radar sensors with different sampling rates. The system must figure out, from noisy sensor measurements, where targets are — even when the sensor might miss them, or report phantom measurements (clutter).

Every file in the project is a distinct module in the tracking pipeline. Here's what each one demonstrates:

<img width="1440" height="1240" alt="image" src="https://github.com/user-attachments/assets/f2c34127-1b81-424d-b299-fca3839f8301" />


## `parameters.m` — System configuration

This is a clean config-as-data file defining the entire simulation: two targets, two sensors, and tracker thresholds. The candidate shows familiarity with:

- **State vector convention** — targets are modeled in 4D Cartesian state `[x, y, ẋ, ẏ]`, a standard choice in tracking
- **Sensor noise models** — separate `range_std` (10m) and `azimuth_std` (0.01 rad) combined into a diagonal measurement noise matrix `R = diag([σ_r², σ_θ²])`, which is the correct form for an EKF measurement noise covariance
- **Clutter modeling** — `false_alarm_density` combined with sensor coverage area to produce a `lambda` parameter for Poisson-distributed false alarms: `λ = ρ × Δr × Δθ`
- **M-of-N tracking thresholds** — `n_tent=3, m_tent=3` (must associate 3 times in 3 observations to confirm), and `n_conf=5, m_conf=1` (delete if fewer than 1 out of last 5 are associated). This is a real, industry-standard track quality logic.

---

## `nextSampleTime.m` — Asynchronous multi-sensor scheduling

One of the more elegant design choices in the project. Instead of stepping every sensor simultaneously, this function finds whichever sensor fires next. With Sensor 1 at T=3s and Sensor 2 at T=7s, they interleave asynchronously. The function:

- Scans all sensors' `last_sample_time + sampling_time`
- Returns the minimum, identifies which sensor fires
- Returns `dt` — the variable time step since the last update

This correctly models how real systems work. The candidate understands that the Kalman filter's transition matrix `F` depends on `dt`, so they parameterize it dynamically rather than assuming a fixed time step.

---

## `moveTarget.m` — Target motion model

Implements the **Constant Velocity (CV) model** using a linear state transition:

```
x_k = F·x_{k-1} + G·v_k
```

Where `F` is the standard kinematic matrix and `G·v_k` injects Gaussian process noise (simulating maneuver uncertainty). Targets have defined `start_time` and `end_time`, so they appear and disappear during the simulation — a key challenge for any tracker.

---

## `generateMeasurements.m` — Sensor measurement simulation

Converts the true Cartesian target position into **polar coordinates** (range + azimuth) as a real radar would report:

```
range = √((x−xs)² + (y−ys)²)
azimuth = atan2(y−ys, x−xs)
```

The candidate then:
- Applies **probabilistic detection** — `rand ≤ Pd` gates whether the sensor detects the target (Pd=0.9 means 10% miss rate per scan)
- Adds Gaussian measurement noise from `mvnrnd` using the sensor's `R` matrix
- Generates **Poisson-distributed false alarms** — `poissrnd(λ)` determines how many clutter returns appear, then places them at random range/azimuth within the sensor's coverage. This is the standard Poisson clutter model from Bar-Shalom's tracking textbooks.

---

## `trackInit.m` — Track initialization with de-biasing

This is technically sophisticated. When a new unassociated measurement arrives in polar form, the system must convert it to a Cartesian state to seed a new track. Naively converting `(r, θ) → (x, y)` would be biased because the nonlinear transformation skews the expected value. The candidate implements the **unbiased polar-to-Cartesian conversion**:

```
x = b⁻¹ · r · cos(θ),   where b = exp(−σ_θ²/2)
```

This is a debiased conversion, not a naive one. They also compute the **converted measurement covariance** `P₀` properly, using the second-order terms that account for the nonlinear spread of the measurement distribution into Cartesian space. This is not introductory material — it's from the Bar-Shalom converted measurement tracking literature.

---

## `PDA.m` — Probabilistic Data Association

This is the core data association algorithm. In a cluttered environment, multiple measurements may fall near a track's predicted position — PDA decides how to update the filter by computing a **soft assignment** across all gated measurements:

1. **Chi-squared gating** — computes the Mahalanobis distance `Dz = vᵀ·S⁻¹·v` for each measurement, where `S = H·P·Hᵀ + R` is the innovation covariance. Only measurements inside the 99% chi-squared gate (`γ = χ²(0.99, 2)`) are kept.
2. **Beta weights** — each gated measurement gets a probability `βᵢ = Lᵢ / (1 − Pd·Pg + ΣLᵢ)`, where `Lᵢ` is the likelihood of the measurement given the track
3. **β₀** — the probability that none of the measurements are correct: `β₀ = (1 − Pd·Pg) / (1 − Pd·Pg + ΣLᵢ)`

These are the exact PDA equations from the Bar-Shalom & Fortmann textbook.

---

## `kalmanFilter.m` — Extended Kalman Filter with PDA update

This implements a full **EKF**, not a linear Kalman filter, because the measurement model is nonlinear (radar measures range and angle, not x/y directly). Key steps:

**Prediction:**
```
x̂_k = F · x̂_{k-1}
P̂_k = F · P_{k-1} · Fᵀ + Q_k
```

**Linearization** — the Jacobian `H_k` of the measurement function is derived analytically:
```
H[0,0] = (x−xs)/r    H[0,1] = (y−ys)/r
H[1,0] = −(y−ys)/r²  H[1,1] = (x−xs)/r²
```

**PDA update** — instead of updating with a single measurement, it blends all gated measurements via PDA weights:
```
x̂ = x̂_k + W · Σβᵢvᵢ
P = β₀·P̂_k + (1−β₀)·Pc + Pspread
```

The `Pspread` term is the innovation spread covariance from PDA, correctly implemented to avoid filter overconfidence when multiple measurements are present.

---

## `trackManager.m` — M-of-N track lifecycle

Manages three track states: **tentative (1)**, **confirmed (2)**, and **deleted**:

- A new measurement that doesn't associate with any existing track spawns a tentative track
- After `m_tent` associations in the last `n_tent` updates → promoted to confirmed
- After the track is confirmed, if it gets fewer than `m_conf` associations in `n_conf` frames → terminated and archived to `confirmed_tracks`
- Tracks termination latency is measured against each target's known `end_time`

---

## `associateTracks.m` — Truth-to-track association for evaluation

A nearest-neighbor position match between ground truth and confirmed tracks (using Euclidean distance with a gate of 100m²). Used only for performance evaluation — not for the tracker itself.

---

## `main.m` — Monte Carlo driver with performance metrics

The orchestrator runs 100 independent trials and computes:

- **RMSE position and speed** across valid (associated) runs per frame
- **Track initiation latency** — how many seconds after a target appears before a confirmed track exists
- **Track termination latency** — how many frames after a target disappears before the track is deleted
- **False track count** — confirmed tracks that don't correspond to any real target

All four plots are generated across frames and Monte Carlo runs.

