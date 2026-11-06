# Codex Handoff

Last updated: 2026-11-02 (Asia/Seoul)

## PCA distribution and shared-axis group profiles (2026-11-02)

- User approved drawing the proposed group-aware views. New output only:
  `analysis/sine35_cable_group_visualization_20261102/` (five PNGs, PCA CSVs/report,
  exact copied selected cases/groups/selection report, README_KO.txt). Old outputs
  and recordings unchanged; no duplicate of the250MiB merged source.
- `scripts/plot_cable_pca_groups.py`: all80,335 eligible force-norm>300 rows fitted
  using ONLY4 cable+4 tension inputs, population z-score then covariance PCA.
  ID/force are eligibility/annotations, not PCA features. Uniform20,000 gray points
  (seed42) for display only, plus all10 selected points and 2D inset zooms.
  PC1/2/3 explain44.8272/42.8326/3.94666%; cumulative2D87.65982%,3D91.60648%.
  Sign fixed by positive largest-absolute loading. CSVs include means/stds,
  component coefficients, eigenvalues, selected PCs and background row indices.
- `scripts/plot_cable_group_profiles.py`: raw40 channel points retained in3D,
  G01blue/G02orange mean guide lines plus channel-local minmax outlines; combined
  displacement/tension profile panels (individuals+means+minmax shading), and
  four shared-axis channel scatter panels each containing both groups.
- Guides are NOT HRM geometry; sample ranges/zoom boxes are NOT confidence regions.
  PCA variance is NOT physical-information retention or estimator accuracy.
  Membership still comes from original complete-link8D0.1mm/20g/300mN criteria.
  No altered coordinates/jitter or synthetic background points.
- 56 related tests pass. Independent SVD selected-PC discrepancy <=2.1e-14;
  20,000 background scores/seed sample set also verified. Source merged SHA
  unchanged; copied case/group/report bytes and provenance strings preserved.

## Channel / displacement / tension scatter in root analysis/ (2026-11-02)

- User requested an unaggregated channel-X / displacement-Y / tension-Z plot
  showing selected cases only, and NEW analysis outputs outside record/.
  Use repository-root `analysis/` for new related outputs, not record/analysis/.
- New folder `analysis/sine35_cable_channel_cases_20261102/` contains two PNGs,
  lossless copies of the latest five-member matched cases/groups and selection
  report, a 40-row `channel_points.csv`, hash manifest, and README_KO.txt.
  G01/G02 correspond to the five-member results (10 cases total), not older pairs.
  Original recordings and historical analyses stay in place. No250MiB duplicate.
- Script `scripts/plot_cable_channel_cases.py`: four points per case, categorical
  E/W/S/N channel1..4, raw motor-derived displacement[mm], native tension[g].
  Color=recorded ID, group=circle/triangle. No averaging, jitter, background,
  or connecting lines. Optional channel-wise view delivered as 2x4 zoom panels,
  with clearly stated independent axis limits; original values unchanged.
- Main plot preserves all8 scalar input values via4points, but overlaps remain.
  Channel spacing is not physical and mixed-unit3D distances are not the match
  criterion. Existing full8D/0.1mm/20g/300mN selection is unchanged.
  Raw CSV timestamp strings (including existing scientific notation) preserved.

## Five-member cable-state groups at 0.1 mm / 20 g (2026-11-02)

- User requested five cases per G group and relaxed cable/tension tolerances.
  Re-ran existing offline tools with exactly five distinct-ID samples per group,
  EACH channel's full group range <=0.1 mm /20 g, individual force norm >300 mN,
  EVERY pair force-vector distance >=300 mN, and existing ID gap>=3 unchanged.
- 80,335 eligible rows; 513,101 radius pairs; 84,734 ID-valid pairs; 71,063
  force-valid pairs; exactly SIX qualifying five-cliques. Independent audit agrees.
  Five candidates share four rows and differ only in five adjacent ID6 frames;
  select the smallest normalized diameter (.9435). Sixth is a separate regime.
  Final: TWO groups /10 unique rows, not six independent cable-state regimes.
- New G01: IDs3/6/9/12/16; max cable span .086091 mm; tension span18.87 g;
  minimum force306.441 mN; every-pair minimum force difference345.962 mN.
  New G02: IDs1/4/7/13/17; max cable span .094334 mm; tension span19.83 g;
  minimum force325.361 mN; every-pair minimum force difference307.219 mN.
  No selected >30deg angle warnings; still awaiting images for visual validation.
- Outputs: `record/analysis/sine35_cable_state_cases_20261102/force300_five_members_c01_t20/`
  (matched_cases/groups CSV, report, README_KO.txt, three plots). G numbering is
  local to this new folder. Existing combined CSV is referenced, not copied.
  Original40 hashes unchanged; all238,401 rows/67 original columns verified again;
  independent audit also checked all10 selected rows and20 within-group pairs.
  No code/model/controller/ROS runtime changes needed for this new search.

## Force >300 mN revision of cable-state examples (2026-11-02)

- User clarified that small-force rows have no meaningful contact location even
  if the experiment file carries an ID. New criteria: EACH sample's aligned
  Kalman XYZ L2 force norm >300 mN (<=300 excluded), and EVERY within-group pair's
  L2 vector difference >=300 mN. These are different checks, not signed sums or
  differences between scalar force magnitudes. Existing0.05mm/5g/idgap>=3 retained.
- Previous36 selected samples contained29 with norm<=300mN; none of the old12
  triples satisfies the new criteria. Preserve those prior outputs as history.
- Added optional norm/pair-distance CLI arguments and full8D radius-graph clique
  search to `analyze_cable_state_cases.py`, avoiding one-neighbor truncation under
  force constraints. Loaded gate replaces (does not combine with) old axis-wise
  `analysis_loaded_reference`. Original combined CSV/labels remain unchanged.
- 80,335 eligible rows;375 close8D/different-ID pairs;283 force-separated pairs;
  ZERO triangles, hence no valid3+member groups. Independent audit agrees.
  Representative selection yields12 distinct two-sample regimes/24 unique rows.
  All selected norms323.754..1810.946mN, pair distances313.239..1439.146mN.
  None has the >30degree angle warning. This is not yet image-based validation.
- New output subfolders under `record/analysis/sine35_cable_state_cases_20261102/`:
  `force300_pairs/` (CSV/report/README_KO.txt/threePNG) and `force300_triplet_check/`
  (zero-result audit). Existing250MiB merged CSV is referenced, not duplicated.
  New G01..G12 refer to NEW pairs, not oldtriples with the same group names.
- Plotter reads merged_csv and thresholds from report; gray background now also
  uses norm>300 eligibility. New CSV reports per-sample force_norm_mN and group
  minimum/maximum norm and minimum/maximum pairwise force distance.
- 36 unit tests pass; all40 original hashes unchanged and all238,401 extracted
  rows/67 source columns reverified. No model/controller/ROS changes. Cases are
  force-contrast-filtered illustrations, not an unbiased occurrence/accuracy estimate.

## Diverse matched cable-state cases and compact CSV extraction (2026-11-02)

- User requested multiple DIFFERENT cable-state regimes, each containing nearby
  cable/tension inputs at different contact IDs, plus one selected-column CSV.
- Input `record/summaries_sine_35deg_both`: 40 CSVs, 238,401 rows. ALL current
  filename IDs match recorded IDs, including 18_right. Do not reapply the old
  October 11 ID1-to-ID18 repair contract to these refreshed files.
- Output `record/analysis/sine35_cable_state_cases_20261102/`: combined CSV 250 MiB,
  78 columns (67 source columns + 5 provenance + 6 analysis flags), manifests,
  12 selected groups / 36 cases, numerical report, README_KO.txt, and 3 PNG plots.
  Every row including missing values is retained; all 67 original column strings
  and nanosecond stamps were rechecked across all 238,401 rows. Source hashes unchanged.
- Full 8D matching: four motor-derived cable displacements [mm] and four
  loadcell tensions [native g], each within-group range <=0.05 mm /5 g.
  Valid/loaded/different-ID eligibility uses historical reference no-load box
  +/-[134.326,126.814,192.670] mN and pairwise ID gap>=3; no live config changed.
  From 115,234 eligible rows the one-neighbor-per-ID/greedy complete-link search
  found 63 overlapping candidate triples, then chose 12 row-disjoint triples by
  input-space farthest-first coverage. This is not a globally optimal/exhaustive
  search. Force/shape differences do NOT rank candidates after eligibility.
- Physical plot axes: (cE-cW)/2 [mm], (cS-cN)/2 [mm], mean(T1..4) [g]. Color is
  recorded ID, size is Kalman aligned force magnitude. NOT actual XYZ space and
  NOT proof of equality in all8 channels. Only the gray display background is sampled.
- G03/G08 include suspect4_left angles (all4,896 rows have |q|>30deg). Retained
  and flagged; don't use their large angle differences as physical shape evidence
  before images are checked. Other10 groups also still need image/history review.
- New offline-only scripts `analyze_cable_state_cases.py`,
  `plot_cable_state_cases.py`, and `test/test_cable_state_cases.py` (15 passing tests).
  No model inference, ROS launch, robot control, or source-recording changes.
  Next: user downloading images; connect by source_file/source_csv_row/source_time_ns.

## IMADA Soft Fy offline contact visualization (2026-11-01)

- User requested contact heatmaps from remaining recorded data. Only
  `record/IMADA_softsilicone_fy_2_good` retains RGB/depth: 1,666 exact-stamp pairs.
  Soft Fz and rigid Fy/Fz have image saving disabled. None of these four bags
  records `/force_id_estimation` or `/estimated_segment_image_geometry`;
  historical GUI overlays/model identity cannot be recovered exactly.
- Generated `record/IMADA_softsilicone_fy_2_good/derived/contact_overlay_20261101/`:
  1,666 annotated PNGs, 55.55 s MP4, six uniform-time examples/overview,
  predictions.csv, frame_index.csv, report.json, README_KO.md and audit hashes.
  All 1,666 geometries reconstructed; 1,570 images have a recent prediction,
  521 loaded frames display a marker, and 1,049 no-load frames hide it. The
  remaining 96 images have no matched valid prediction (record-start warm-up).
- Explicit recomputation choice, NOT verified historical selection: current
  GUI-default cable_loadcell_angle masked force-KalmanNet/location-Transformer,
  30x26 window; current inclusive XYZ no-load box +/-200 mN, ID band +/-2.
  Raw-topic CSV receipt-order replay uses FeatureHistory; summary nearest-time
  joins are NOT the inference inputs. Image association uses preceding input
  source stamps, not original GUI availability/timer phase. No ground-truth
  force/contact labels are used as inputs; output is not an accuracy claim.
- Added offline-only scripts `offline_contact_inference.py`,
  `offline_contact_geometry.py`, `render_recorded_contact.py` and 42 tests.
  RBSC uses saved estimator config, crop-local K, zero ROI offset and fresh
  filters. Contact IDs exclude fixed physical segment0. No ROS nodes, robot
  commands, changes to original recordings, or live recorder changes were made.
- PNG/MP4 frames and probabilities/argmax/age/no-load behavior verified.
  Future live recorder still needs an explicit separate change to retain
  force-ID predictions/geometry/GUI contact overlays; not silently added here.

## Git staging exclusions (2026-10-31)

- Exclude local `/src/estimation_pkg/models/`: about30.6GiB, mostly validation
  NPZs/workbooks plus bundles. No files removed. New-PC inference requires
  separately copied `hrm_bundle.pt`; instructions added to force_id_inference.md.
- User approved removing blanket `scripts` ignore. Retain command/diagnostic/
  figure-generation scripts and regression tests; output under docs/figures,
  build/install/log/record and caches remains ignored. Added coverage exclusions.
- Existing .gitignore changes otherwise preserved. Dry-run staging checked;
  no real git add, commit, index removal or history rewrite. Existing tracked
  deletions (legacy models and docs/analysis files) remain user changes; ignoring
  paths does not suppress those deletions or reduce existing .git history size.
- Remaining present modified/untracked candidate files total about1.9MiB, not
  an estimate of push size. Main .git directory was about32GiB before this change.

## ON-HRM force convention / compliance sign correction (2026-10-31, current)

- After reporting divergence, user confirmed stopping and authorized another
  correction. Public forces remain ON the HRM in hrm_base, with node input
  `F_env=-prediction`. Since `goal=nominal+offset`, physical compliance requires
  `M*offset_ddot+B*offset_dot+K*offset=F_env-F_target`. Restored the wrapper's
  `computeAdmittance(-f_desired,-f_env,dt)` adapter to the legacy filter algebra.
  The previous direct desired-env rollback below is superseded. It verified
  arithmetic but could reverse compliance under the current force definition.
- Position PID, motor mapping, source-image dt, matrices/solver, resets, force
  input sign, GUI target signs and hardware guards are NOT modified. No deadband
  or offset/velocity cap added. User-edited source M=.05/B1/K20 is retained;
  earlier live M=.1/B15/K50 is not automatically preserved by a node restart.
  Read actual parameters before testing. Source defaults are not validated gains.
- During the isolated build, concurrent operator edits temporarily left
  FORCE_TIMEOUT_SEC empty, then set it to1.0 (outside the current <=.25 guard).
  User explicitly approved restoring .25 and sign-only work; restored only that
  constant. Kept operator edits MAX_OFFSET_M=.002/MAX_VELOCITY_M_S=.001; these
  constants are unused in the current path and do NOT reinstate motion clamps.
- Concurrent operator edits also changed source position KP20.55→25.55,
  max angular speed40→25deg/s, position feedback timeout1→.25s, motor velocity
  profile50→25. Preserved them without editing PID logic. They are included in
  the new build; do not claim the whole running experiment differs only by sign.
- Read-only capture after the divergence report saw only three paired frames:
  latest Y target=-100mN, correctedFy=+550.2mN, nominalY=-11.443mm,
  offsetY=-11.572mm, effectiveY=-23.014mm; correctedFz=+588.0mN.
  This was not necessarily the user's reported zero-target/near-zero-force moment.
  User later estimated near-zero force as about50mN. Neither that nor UNLOADED
  means numeric zero. Constant50mN gives equilibrium2.5mm atK20 or1mm atK50.
- Pre-change source files/tests/docs and installed binary backed up at
  `/tmp/hrm_admittance_pre_compliance_20261031.vsgYEo`. This backup contains the
  problematic direct legacy sign, not a validated safe rollback. Do not restore
  it or an older bounded-offset version automatically if a hardware test fails.
- Validation: isolated Release build `/tmp/hrm_compliance_sign_20261031.Jare5a`
  passed all six C++ test targets (11 wrapper tests including 108 combinations
  of gains/axis/contact/target/bias, plus12 signed perturbation cases). Python
  GUI/diagnostic regressions229 passed. Domain79 localhost fake-feedback smoke
  passed391 paired diagnostics, zero actuator messages, output disabled, including
  Y=-.01 nominal with exactzero state/forces, both force signs and existing guards.
  Normal workspace build completed, and its six C++ test targets also passed.
  Hardware controller remains stopped; no automatic restart or motor commands.
- Physical stability caveat: sampled passive-spring/ideal-position analysis is
  illustrative, not an identified HRM model. With M=.05/B1/K20 and Ke100N/m,
  the corrected sign can converge at30Hz but diverge at20Hz or with one-frame
  force delay. Internal numerical substeps do not remove feedback delay. Do not
  infer physical convergence from algebra or the selected simulation cases.

## Legacy admittance force-error sign restored (2026-10-31, superseded)

- User explicitly requested the legacy convention after comparing the original
  humble controller with the newer double-input sign adapter. Removed that adapter:
  `computeAdmittance(f_desired, f_env, dt)` now receives both arguments unchanged.
  Thus `delta_force = F_target - F_env`, and `x_t = x_desired + delta_x`.
  This supersedes older entries below describing `F_env - F_target`.
- Node input `force_sign=-1`, hrm_base/N units, Y/Z matrices, M/B/K defaults,
  Position PID/rate integration, source-image dt, entry resets, hardware guards
  and the filter's singular solve/numerical substeps are retained. This is NOT
  a full rollback to old 2D geometry or fixed-step unsafe numerical integration.
  Equal target/environment forces still cancel; a nonzero offset is not instantly
  reset when both forces become zero. Unloaded classification does not zero force.
- Read-only observation before editing found nominal Y=-10 mm was retained but
  nonzero corrected force added an offset: one matched sample had Fy=+307.3 mN,
  offsetY=+10.533 mm, effectiveY=+0.533 mm, measuredY=+0.222 mm. That capture used
  M=.15/B20/K30; an earlier interval used .15/25/2. Do not describe the observed
  incident as proven zero-force drift or infer a sole sign/model cause from it.
- The user confirmed stopping; process presence is checked before replacing the
  normal binary. Testing uses an isolated build and private ROS domain with motor
  output disabled and actuator topic remapped. No automatic hardware restart.
- Validation: isolated Release build `/tmp/hrm_legacy_sign_20261031.OEEH6V`
  succeeded; all six C++ targets passed, including nine wrapper regressions.
  GUI/diagnostic regression tests: 229 passed. Private domain 79 fake-feedback
  smoke passed with 395 paired diagnostics and zero actuator messages. Covers
  positive/negative Y/Z, input sign exactly once, zero-state nominal hold,
  retained nonzero state on zero force and all existing feedback/hardware guards.
  These validate code behavior, not physical contact stability or model accuracy.
- Normal workspace build completed after confirming the hardware controller
  process exited; all six C++ test targets also passed in the normal build.
  Controller process remained stopped after validation. No automatic
  controller/GUI restart or real motor actuation was performed.
- The earlier request to simplify the force staircase to 100..500..100 was
  interrupted before edits. Do not claim that script was updated in this task.

## M/B/K tuning assistant and read-only diagnostics (2026-10-29)

- User wants informed M/B/K selection for slow/nonconvergent force response.
  Added pure `admittance_tuning.py`, offline `AdmittanceTuningDialog`, and a compact
  Tune button in the existing 48 px panel. Candidate B=Fe/v, M=B*tau,
  K=budget/distance (K=0 in force mode); initialized from confirmed values, not
  autotuned. Current/candidate metrics distinguish free-model decay from M/B
  velocity time constant, and warn about spring residuals and bias drift.
- Use candidate only stages editors; existing explicit Apply/mode guards remain.
  No controller, force-sign, default-gain, mode, Enable or axis-selection changes.
  Shared Y/Z remain active: Fz*=0 does NOT disable Z compliance.
  No automatic robot motion or GUI restart.
- New `scripts/inspect_admittance_tuning.py`: read-only three-topic source-stamp
  join, per-condition tail-error RMSE, spring-balance/inner-lag/drift hints and
  optional new CSV. Missing pairs, short windows and disabled output are explicit;
  analysis thresholds are not controller limits. Force uses the controller sign
  and N→mN conversion, not raw prediction sign or independent force ground truth.
  Legacy PositionControl reports only pan PD gains; hold both PD axes fixed.
- Updated controller README defaults to current source .15/20/30 (documentation
  only; actual running values may differ). Procedure is in that README and
  docs/CONTROL_READINESS.md.
- Validation: 265 focused/regression tests passed (candidate math, offscreen Qt,
  telemetry analysis, existing M/B/K/force-target/PD controls and staircase CLI).
  Isolated localhost ROS domain 82 fake telemetry passed exact-stamp pairing,
  spring-balance evidence and disabled-output caveats; no actuator/services used.
  An 8 s read-only live capture received no diagnostic messages, so physical
  convergence was NOT validated. Installed symlink imports of new GUI modules
  work without rebuilding; user-controlled GUI restart is needed to see Tune.

## Terminal admittance force staircase (2026-10-29)

- User requested Fy/Fz separately:0,200,300,400,500,400,300,200,0mN every3s.
  New standalone `scripts/admittance_force_steps.py --axis fy|fz`, optional
  `--sign -1`, `--interval`, `--node`, `--timeout`, and ROS-free `--dry-run`.
  Holds all9steps including initial/final0 (~27s plus acknowledgement latency).
- Uses existing parameter service, writes ONLY selected target as N; other axis
  unchanged. GUI/physical force-on-HRM convention, no implicit model-sign flip.
  Reads vector limit, checks sequence, awaits ACK and stops on error. No controller
  mode/enable, gains, model, position reference or control algorithm changes.
- Ctrl+C/SIGTERM/error after a write attempt best-effort requests selected-axis0;
  failure is explicit. Zero is NOT motor disable/hold; SIGKILL/network loss can
  leave the last target. User must verify setup/contact/sign before real use.
- Validation:38 mock tests passed; private localhost ROS domain81 fake parameter
  server passed both9-step signed sequences, other-axis preservation and Ctrl+C
  selected-axis0 ACK. No real robot execution or live parameter writes performed.
  See robot_control_pkg README for commands; no rebuild required.

## Position rate-PD restored after free-motion oscillation (2026-10-29, current)

- User explicitly requested reverting the immediately preceding Position change.
  Restored `u=Kp*e+Kd*LPF(de/dt)` as angular rate [rad/s], then
  `q_next=clamp(q_previous+clamp(u,+/-speed)*image_dt,+/-bend_limit)`.
  This supersedes ONLY the 10/27 rad/update Position change below. Admittance
  uses the same inner Position controller, so its inner PD changes too.
- Read-only 12s observation before stopping: Position mode, P20.55/I0/D10,
  image rate ~30Hz/dt33.34ms, command rate near +/-40deg/s on Y71.4%/Z58.9%
  of steps. Command saturation is not actual motor-speed measurement; code
  regression is a plausible oscillation cause, not proof of sole hardware cause.
- Numeric source defaults unchanged: P20.55/I0/D5, D tau0.05s,
  angular speed40deg/s and motor profile35. Runtime GUI D10 was not persisted;
  restarting uses source defaults unless explicitly overridden. Read gains first.
  GUI tooltips now use P rad/(m*s), D rad/m. No gains auto-retuned.
- Admittance 6x6 M/B/K, force signs, force gating, PID scalar implementation,
  control_node frame handling and all motor/loadcell protections untouched.
- User confirmed stopping before normal install build. Both isolated build
  `/tmp/hrm_rate_pd_restore_20261029.2Ij8WT` and normal colcon build succeeded;
  all6 C++ test targets passed in both, including23 Position cases, plus21 GUI
  mock tests. Position fake-feedback smoke(domain78) passed with output disabled
  and actuator remapped: actual actuator messages0. Tests include elapsed-time
  invariance at15/30/60Hz, source dt, frame gaps, runtime gains and retained guards.
- Admittance fake-feedback smoke(domain79) also passed:369 paired diagnostics,
  actuator messages0, force signs/targets, unclipped offsets and existing guards.
  Its test helper now waits for same-stamp Position diagnostics (maximum3s), fixing
  a cross-topic callback-order race without omitting samples or changing runtime.
- No real motor movement or automatic controller/GUI restart. Normal install is
  ready for user-controlled restart; physical oscillation must still be checked.

## Disconnected skeleton visualization (2026-10-29)

- User asked how fragmented body masks/skeletons are processed and requested overlays.
  Offline feasibility front end preserves connected components area>=16, Lee thinning,
  weighted 8-neighbour double-sweep main paths, minimum path points=1, greedy nearest
  endpoint ordering from image-bottom base. No hidden pixels or XYZ are inserted.
  Valid-depth points feed the shared origin-constrained 3D quartic; gap parameter
  distances are endpoint chords, not observed hidden arc lengths. Not unchanged live preprocessing.
- New `scripts/prepare_segmentation_skeleton_figure.py` reads only frozen NPZ/PNG/CSV.
  Main figure folder now has `B_visible_body_skeleton.png`, native skeleton PNGs,
  RGB/mask overlays, `source_data/example_skeleton_pixels.csv`, and
  `skeleton_visualization.json`. Main A-D and original results are unchanged.
- Example clean/thick30/multi3_30 support counts146/68/31 match valid observed pixels.
  Three gaps has4 paths, base-to-tip2/8/3/18 pixels with raw IDs4/3/2/1.
  P1-P4 denote path traversal order, NOT anatomical segment/joint IDs.
  First observed pixel is(v295,u189); short pieces can thin to1-2pixels.
- Portable MATLAB adds `B_skeleton` selector and `preview_skeleton.png`; default all
  now10figures. Added exact skeleton pixel Excel sheet and supporting PNGs/manifest.
  No MATLAB/Octave runtime available: static/data checks only; previews are Python renders.

## Portable MATLAB segmentation figures (2026-10-28)

- Copyable bundle: `docs/figures/segmentation_occlusion_matlab_20261028/`.
  Single MATLAB file `make_segmentation_figures.m` (helpers merged as local functions)
  plots A/B/B_skeleton/C/C_raw/D/D_position/D_angles/
  joints/ABCD from included PNG images and `data/segmentation_figure_data.xlsx`.
  No repository, Python, model, ROS, camera, or external original paths needed at runtime.
- Excel preserves all104 paired frames and1872 joint deltas, original metric units,
  exact timestamp strings and signed angles. Builder `scripts/package_segmentation_matlab.py`
  verifies Excel round trip, per-frame FK/q differences, perjoint RMSE/MAE and source hashes.
- Targets base MATLAB R2020b+, no image/statistics toolboxes. MATLAB/Octave is absent
  on this machine: data/static verification only; do not claim MATLAB runtime rendering.
  `preview_reference.png` is the earlier Python figure, explicitly not a MATLAB render.
- User requested no PDF/SVG and editable C/D plots: outputs PNG/FIG only in `output/`.
  C/D use native plot3/scatter/plot/patch graphics, never screenshot images. Third
  argument `saveFiles=false` opens plots without writing files or creating directories;
  result includes data, figure handles and validation. ZIP was refreshed accordingly.
- Latest B panel now compares the original saved Unoccluded mask and Three gaps
  mask side by side (RGB is already in A). Updated MATLAB, PNGs, caption and ZIP;
  no new inference or data changes. Older PDF/SVG assets remain unchanged.

## Segmentation Results figure assets (2026-10-28)

- User requested actual A-D assets following `docs/Screenshot from 2026-10-28 15-06-27.png`.
- New bundle: `docs/figures/segmentation_occlusion_20261028/README_KO.md`; composite and
  separate PNG/SVG/PDF, Korean guide/English LaTeX caption, selected original RGB-D/masks/NPZ,
  C coordinate CSVs, D all104-frame and1872joint-delta source CSVs plus perjoint supplement.
- No training/inference/ROS/shape rerun. A cases regenerated by original seeds and pixel-exact
  checked against frozen comparison sheets; B uses original saved U-Net masks unchanged.
- A-C use archive midpoint frame822/source1790484556946270508 (depth filename821, exact stamp).
  Selected most bent clean-tip transverse among3 previously saved mask examples, not by error.
- C main uses saved length-normalized curves + FKchains/tips. Separate raw observation panel
  uses actual depth points and curves divided by recorded lengthscale; never silently scale
  measured points. Dashed s intervals between observed components are not exact occluder widths.
- D multi3_30 only:40straight/64bent; means .413/.554mm and1.265/1.477deg meanframeqRMSE;
  <=1mm38/40 and58/64. All outliers shown; same-method clean reference, noGT/accuracy claim.
- Generators: `scripts/prepare_segmentation_figure.py`, `scripts/prepare_segmentation_panel_d.py`.
  D audit recomputed104 NPZ pairs/624metrics/1872deltas and213sourcehashes unchanged.

## Segmentation occlusion paper handoff (2026-10-27)

- User requested a standalone Korean technical/results Markdown for another
  manuscript-writing AI: `docs/SEGMENTATION_OCCLUSION_PAPER_HANDOFF_KO.md`.
- Audited sibling Segmentation project's saved V2 short-fragment RGB-D results,
  not new training/inference. Original archive45deg1 RGB-D64frames plus new black
  straight40frames; source archive now under sibling `datasets/`, historical
  main `record/...` paths in protocol no longer exist. Exact-source-stamp pairing.
- Recomputed all728 U-NetV2 shape NPZ metrics; exact agreement with saved CSV.
  Includes all6 occlusioncondition pose/q summaries, perjoint18 RMSE/MAE for
  multi3_30, segmentation Dice, formulas and figure/source paths. Independent
  table review verified156 displayed numeric entries.
- Crucial: same-method same-frame clean estimate is reference, NOT AprilTag GT
  or original OpenCV-mask shape. Known syntheticoccluder depth excluded; nohidden
  depth reuse, noautomatic realoccluder-depth rejection validated. Offlinefront
  end differs; production3Dquartic/chord/DH/FK core reused, temporalfiltersOFF.
- Multi3_30 FKmean .413/.554mm, q18 frameRMSEmean1.265/1.477deg (straight/bent).
  <=1mm fractions95.0/90.625%; not universal1mm accuracy. Includes p95/max,
  initialminpath8→final1 development caveat and V3seg-only separation.
- No robot/controller code, running nodes, recordings or learned models changed.

## Legacy admittance / angular-increment restoration (2026-10-27, historical)

- User compared the original humble filter/controller and explicitly requested
  restoring prior yielding behavior. At that time this superseded earlier rate-PD
  and bounded admittance notes below. The Position portion was reverted on10/29;
  see current entry above. No live robot process, mode, gate or parameter changed.
- Restored generic6x6 M/B/K branches and legacy desired-env force error. Removed
  ±5mm/2mm/s clamps and their ROS parameters. Full coupled symmetric PSD matrices
  supported; inactive axes and rank-deficient consistent systems safely solved.
- Public forces remain force ON robot in hrm_base. Controller passes (-target,
  -environment) to legacy filter, keeping its RHS=environment-target, displacement
  additive to nominalgoal and existing once-only training sign conversion=-1.
  This adapter deliberately preserves current physical convention, not a blind
  restoration of old sensor-axis/sign mapping.
- Legacy acceleration->velocity->position integration uses conservative internal
  substeps when current M/B/K and source dt require it. No response clipping or
  parameter retuning. Current .05/5/2 at30Hz is numerically unstable if the old
  explicit update is applied in one full frame; substeps address numerical error,
  not a guarantee of closed-loop stability.
- Position PD output is rad/update again, directly added to previous command.
  Separate40deg/s angular slew guard, bend limit, I0, Dfilter, fresh-frame pacing,
  camera recovery and motor/loadcell/force guards retained. Current numeric user
  P20.55/D5, M.05/B5/K2 and motorprofile35 remain. Their previous rate-PD tuning
  is NOT equivalent; verify with motors disabled and retune before experiments.
- GUI PID unit tooltips updated; gain controls/force targets stay unchanged.
  Invalid per-frame vector-as-%f logging replaced by typed Y/Z DEBUG logging.
- Validation: isolated robot_control_pkg build passed at
  `/tmp/hrm_legacy_control_20261027.cgUNli`; all6 C++ gtest targets passed;
  GUI PD/M/B/K/target tests78passed. Fake-feedback Position(domain78) and
  Admittance(domain79) smoke tests passed with motor_output_enabled=false and
  remapped actuator topic; actuator messages0. Verified unclipped >5mm/>2mm/s,
  force signs/targets, frame gaps, legacy angular increments and retained guards.
  Normal install and running robot were NOT replaced. Safely stop motion and
  disable output before building normal install/restarting for deployment.

## GUI M/B/K settings and horizontal controller grouping (2026-10-27, latest)

- User resolved contact/reaction force sign confusion and explicitly requested
  NO filter/sign changes. Only add GUI M/B/K controls and visual separation.
  Request called stiffness D_d; existing backend name K_d is retained and explained.
- New `gui_py_pkg/admittance_parameters.py` edits existing shared Y/Z scalar
  `admittance/M_d` [kg], `B_d` [N s/m], `K_d` [N/m] through parameter services.
  This does NOT restore a general 6D solver or introduce per-axis matrix editing.
  No control_node/admittance_filter, force sign, limits or defaults changed here.
- Read-before-edit/apply; exactly three DOUBLE parameters written atomically on
  explicit Apply, ACK plus verifying readback required. Finite/nonnegative/not-all-
  zero validation; unedited server precision retained. Discovery and 2s refresh
  read only; refresh preserves pending edits. Timeout/offline states prevent
  overlapping writes. Runtime settings only; no automatic mode or enable change.
- Existing backend mode4 restriction remains: M/B/K Apply disabled with
  `Leave Admit` while Admittance active. Leave mode manually and Read/refresh,
  then Apply. Separate target Fy/Fz controls still allow live changes as before.
- Motor control settings row: compact Position PD left, vertical separator,
  two-row M/B/K panel right. Horizontal separator before existing target-force row.
  PD service behavior unchanged. Motor layout has no scrollbar at tested sizes;
  old six-plot regression is NOT reintroduced (two plots /10Hz display retained).
- Validation uses fake/offscreen ROS services and isolated GUI build only.
  No live GUI restart, controller parameter writes, mode changes or motor commands.
  Normal GUI symlink install loads new Python module on next safe GUI restart;
  stop motion and recording first because GUI owns component subprocesses.
- Full functional GUI suite595passed (legacy lint templates excluded), including
  27 M/B/K service/readability tests; isolated gui_py_pkg build, new-file Flake8,
  py_compile and diff-check passed. Populated mock screenshots at1366x768,
  1500x900,1920x1080 inspected in `/tmp/hrm_mbk_gui_b9ips1nl`. At1366, PD475px
  and M/B/K317px fit side by side. Compact numeric displays use4decimals; the
  M/B/K panel keeps unedited original parameter precision when applying.
- Existing layout/camera autostart tests now respect user's current per-component
  auto_start choices (force_id and record true), retaining schedule/delay checks.
  `system_components.json` was not modified by this task.

## GUI plotting rollback after rate complaint (2026-10-27, latest)

- User reported severe serial/prediction Hz regression and requested reverting
  graphs if responsible. Independent read-only ~10s subscriber measured FT200.03Hz,
  loadcell200.03Hz, force-ID30.00Hz, angles29.99Hz, motor242.4Hz. Max gaps8/8/35/43/
  25ms respectively. GUI instantaneous CPU117.4%; actual producers were healthy.
- Six full Matplotlib subplot redraws shared Qt with ROS spin. Synthetic equal-size
  canvas baseline mean27.58ms/p95 28.75ms, exceeding requested25ms refresh. GUI
  spin_once also handled just one callback per5ms (max200/s vs >640/s input).
- Restored TWO vertically stacked plots: all measured/predicted forces on top,
  torques below. Supersedes prior2x3 requirement below. Retains gray grids, fixed
  legends, differentiated prediction lines, frame caption, nine channels and NaNs.
  Confirmed red Fy/Fz targets share force plot; target GUI/sign logic unchanged.
  Force/torque scales grouped separately, xlimits fixed atinit, skip unchangedylimits.
- Graph-only refresh100ms (10Hz); 50display samples span~5s, not stored dataset
  cadence. ROS timer remains5ms but drains atmost16 callbacks or2ms budget each
  tick (running callback notpreempted). No subscriptions/QoS/source rates changed.
  No control/force/gain/recording changes, no motor/settings writes or GUI restarts.
- New synthetic two-axis draw mean13.44ms/p95 15.29ms; this is not post-deployment
  liveGUI performance confirmation. Current running GUI still has old loaded code.
  Operator must safely stop motion/recording before restarting its owned components.
- Fake Qt receive test (no ROS network, 640 messages/s, new plot cadence): old
  single-callback poll p95 delay836ms vs bounded batch13ms. This demonstrates Qt
  receive capacity improvement, not live DDS/control timing guarantees.
- Tests adjusted for two-axis semantics, separate `test_gui_ros_drain.py` covers
  count/time budgets, shutdown, and receive-only callback bursts with fake ROS.
  Full functional GUI suite561passed (legacy lint templates excluded), compact
  tests65passed, new drain tests5passed; isolated GUI build and py_compile passed.
  Normal source symlink install loads edits on safe GUI restart; no rebuild needed
  for these Python-only edits. No old recordings or saved numeric data altered.

## GUI admittance Fy/Fz targets and sign/log audit (2026-10-27, latest)

- Added `gui_py_pkg/admittance_target.py`: compact Motor control row below Position
  PD, signed Fy/Fz in mN, Read/Apply/status and read-only sign/max-force norm.
  Explicit Apply atomically writes only target_force_y_N/target_force_z_N to
  `/robot_control`. No motor/gate/mode writes; applying during Admittance can
  immediately affect motion. Settings runtime-only; startup/discovery reads only.
- Post-ACK readback required; edited/rejected values retain last confirmed plot
  targets, dispatched/unconfirmed writes and offline hide them. Full precision
  retained for unedited fields. Read-only 2s refresh follows external changes while
  preserving pending edits; timed-out writes never overlap/retry automatically.
- GUI Fy/Fz plots gain thin red target overlays, separate from the nine data lines.
  Convention is force ON robot: plot target = target_N / force_sign * 1000.
  With sign=-1, +200mN input draws -200mN on unchanged unnegated prediction plots.
  Legacy-only sensor-frame prediction hides target overlays; unified-source stale
  prediction stays NaN while known confirmed target remains. Target included only
  in corresponding axis scale, not numeric buffers/topics/CSV. Targets visible
  outside Admittance mean configured targets, not an active-control indicator.
- Motor status shares Motor control heading; PD inputs/buttons use23px rows to
  keep added target row and three previews within1366x768 without motor scrolling.
- Force sign audit: `control_node.cpp:2070-2072` multiplies incoming XYZ once by
  `admittance_settings_.force_sign`, default `control_parameters.hpp:87 = -1`.
  `control_node.cpp:2161` passes f_env into admittance; filter uses Y/Z only with
  force error `F_env-F_target`. No control algorithm, policy or safety change here.
- Actual user log `/home/daeyun/.ros/log/robot_control_141976_1793068929204.log`
  around line1585 alternates ACTIVE with/without offset/velocity-limited suffix.
  Whole status-string changes trigger `set_position_status` logging; no WAITING/
  FAULT observed in that log. Combined flag cannot distinguish axis or bound.
  ID display-only policy assumes physical tip contact, not classifier correctness.
- Read-only runtime check: sign=-1, ID gate=false, maxoffset=.005m,
  maxvelocity=.002m/s, maxforce=2N, targets0/0N. During audit mode1 (Kinematics),
  motor_output_enabled=true. Never send actual targets/restart live GUI for tests.
  Preserve user gains20.55/0/5, speed40deg/s and motor profile75%; no edits to them.
- Testing uses fake/offscreen ROS services and isolated GUI build in
  `/tmp/hrm_position_sine_20261026.nRlJ8d`; no physical control settings changed.
  All 558 functional GUI tests passed (legacy lint templates excluded), including 32
  target-service tests and 67 compact/integration tests. New module/tests flake8,
  py_compile and isolated gui_py_pkg build passed. Mock-only previews inspected at
  1366x768/1500x900/1920x1080 in `/tmp/hrm_admittance_target_gui_hpui1nz3`.
  Normal `build/gui_py_pkg/gui_py_pkg` links the source package directory, so the
  new module is visible through the existing symlink install. Stop motion/recording
  safely before GUI restart; closing it may stop its owned components.

## GUI force/prediction and torque plots (2026-10-27, latest)

- User requested individual Fx/Fy/Fz comparison plots, light gray horizontal
  major-tick grids, and denser/differentiated prediction lines. They then explicitly
  corrected layout to TWO ROWS × THREE COLUMNS: top fx/fy/fz (each with prediction),
  bottom tx/ty/tz. Do not revert to three rows with force/torque columns.
- `gui_node.py::init_fts_plot`: one existing canvas, six independently scaled axes;
  public order fx/fy/fz/tx/ty/tz, nine line/data rows remain measured6+predicted3.
  Measured solid1.5pt, prediction dense dash(1.6,1.0)/1.1pt, darker same hue.
  Fixed upper-right legends, compact component/unit titles, horizontalgray guides.
  Shared50sample buffer and25ms animation unchanged; no new ROS inputs or timers.
- Per-force limits include only its own predicted/measured component. Stale/invalid
  prediction remains NaN gaps, not held or zero-filled. Frame caption explicitly
  distinguishes sensor measurement vs unified hrm_base prediction, resets for
  legacy source; no frame conversion/sign inversion or numeric/topic/CSV change.
- Short-screen fallback only around graph canvas: minimumheight160, verticalscroll
  if viewportshort, no horizontalscroll. Offscreen1366x768 uses this graph fallback;
  1500x900 and1920x1080 show all six. Motor/image layouts and controls unchanged.
  Ylabels changed to compact titles to avoid narrow-column label overlap.
- FunctionalGUI suite518passed (linttemplatesexcluded); new tests cover mapping,
  isolatedscales, tickgrid, dense/thin/darkerlines, fixedlegends, NaN/recovery,
  framecaption andgraphbounds at3sizes. Isolated gui_py_pkg build passed in
  `/tmp/hrm_position_sine_20261026.nRlJ8d`. Synthetic screenshots inspected in
  `/tmp/hrm_force_graph_2rows_final_vnwops7s` (before final4px leftmargin increase).
- No live GUI/node/process/motor actions or runtime settings changed. Tests mock
  ROS and prevent process launches. Normal install egg-link points to build package,
  whose gui_node.py resolves to source: GUI restart loads edits without rebuild.
  Stop motion/recording safely before closing GUI (owns childcomponents).

## Separate command/pose CSV for control trials (2026-10-26, latest)

- User explicitly requested commands and observed tip pose together for Kinematics
  and Position, in a separate CSV rather than adding controller fields to summary.
  New automatic Stop/export outputs: `csv/control_tracking.csv` and
  `control_tracking.schema.json`. Existing 268-column summary and topic CSVs
  remain unchanged; no old recordings were regenerated, replaced or deleted.
- Additive `custom_interfaces/msg/ControlReference.msg`, topic `/control/reference`:
  generation stamp + feedback image stamp + mode + IK pan/tilt rad + nominal/effective
  Cartesian goal m + gate flag. Produced in IK command generation and successful
  feedback-step diagnostics. No control/IK/PID/safety algorithm changed in this task.
  Gate is NOT actuator acknowledgment; subsequent hardware guard may block output.
- Recorder adds reference and Position sine reference/status to optional selected
  topics, not required startup gates. New `control_tracking_csv.enabled=true`;
  export/finalizer/status integration independent of summary's training-data gate.
  Export function and ROS CLI `export_control_tracking <existing_csv_directory>`
  exclusively create new files, refusing overwrites. No extra live subscribers.
- Rows anchor every reference. Kin uses command time; Position/Admittance use the
  feedback image time. Independent streaming nearest-source cursors per basis
  avoid mode-switch backwards clock offsets. No age limit (existing user policy);
  each group keeps actual stamp, signed time difference, frame and validity. Missing
  or wrong-frame data stays blank, never zero/identity. Clock/latency correction,
  interpolation, long-gap rejection and calibrated accuracy claims are not added.
- Includes estimated tip boundary XYZ, observed-angle FK point + FK TF pose, and
  estimated last-link DH orientation (center_19 rotation ONLY, not its position).
  Kin Cartesian columns blank; feedback modes nominal and effective target both
  stored plus target-minus-FK Y/Z error. Joint aggregate command rad is not Euler.
  Contact label copied from session; source time and elapsed_s retained.
- Event-only caveat: Kin sine updates are recorded normally. A single fixed IK
  command must be issued AFTER Record starts; it is recorded once, not periodically
  replayed throughout a hold. No guessing pre-record goals or legacy headerless
  command modes. Missing reference produces header-only `no_commands`, with a
  recorder status note; it doesn't block ordinary data recording.
- Validation: isolated custom_interfaces/robot_control_pkg/record_pkg builds in
  `/tmp/hrm_position_sine_20261026.nRlJ8d`; recorder functional suite 490 passed,
  including 23 focused +9 integration cases and actual offline rosbag decoding;
  producer/FK source checks 9 passed; all five C++ functional suites passed.
  Full controller CTest still reports pre-existing formatting/lint failures
  (flake8, lint_cmake, pep257, uncrustify), not fixed as unrelated scope.
  New-files targeted Flake8 clean; installed export CLI help verified.
- `test/control_reference_smoke.py` passed private localhost DDS84, explicit
  isolated binary, motor gate false and remapped actuator topic:25 references,
  24 FK samples, zero actuator publications. Absolute/relative IK, Position and
  Admittance units/modes/timestamps/target values checked. Log
  `/tmp/hrm_control_reference_smoke_wsc_mibn`. No live robot commands, parameter
  changes or restarts; running normal controller binary was not replaced.
- Deploy after safely stopping motor motion/controller and recording/export:
  `colcon build --symlink-install --packages-select custom_interfaces robot_control_pkg record_pkg`,
  then source install/setup.bash and restart controller + Data recorder.
  Do not attempt to load new-message binary with old custom_interfaces overlay.

## Y/Z sine reference speed default 10 mm/s (2026-10-26, latest)

- User confirmed the Start rejection `2*pi*amplitude/period <= max_speed_m_s`
  and requested10mm/s. Changed PositionSineConfig::speed .005 -> .010m/s.
  This is a per-axis reference peak/approach limit, not position tolerance or
  measured actuator speed. Defaults A=.010m/T=20s/phase0,90 and PD gains unchanged.
- T=20s allows up to31.83mm amplitude under the speed condition; combined
  workspace validation remains. User can reduce amplitude/increase period.
- No launch/GUI override shadows the default; GUI Start only sends Y/Z/period.
  Existing smoke's explicit .005 override intentionally tests slower operation.
  Older5mm/s notes below are historical. Live node unchanged; after stopping
  sine the user may set `motion.position_sine.max_speed_m_s` to0.01 at runtime,
  or rebuild/restart robot_control_pkg to obtain the new persistent default.
- Validation: all10 PositionSine unit tests passed in the isolated build,
  including20mm/20s acceptance at10mm/s and rejection with a5mm/s override.
  No normal controller binary rebuild or live parameter/motion changes.

## Position motor speed and transient-feedback recovery (2026-10-26, latest)

- User reports Position much slower than Kinematics, Kp-induced overshoot,
  and goal rejection after an apparent recognition loss. They also requested
  negative predicted force for admittance; `admittance/force_sign=-1` already
  applies exactly once at the input. No extra negation was added.
- Confirmed active motor profile mismatch: Kinematics50 vs Position/Admittance10.
  New integer `position_control/motor_velocity_profile_percent` defaults50,
  startup/runtime validated1–100 before any atomic batch mutation. Both feedback
  modes use it; software hold remains10. TCP passes it unchanged; source firmware
  scales by VELMAX/VELRES. This removes a5x profile discrepancy, NOT a measured
  5x improvement or proof that closed-loop motion now matches IK duration.
- PRESERVE operator's header edits max angular speed20deg/s and freshness1.0s.
  P5/I0/D.05 unchanged. At10mm error P requests only2.86deg/s. Angular-command
  integration can lead slow actual motors even with PID I=0; no encoder-following
  anti-windup was added and physical overshoot is not claimed solved.
- Source dt integration cap is now independently0.25s. On longer gap or explicit
  force/vision pause recovery, re-anchor time/reset PID history without resetting
  goal, IK command, encoder origins, admittance offset, or sine phase; the next
  normal frame resumes. No catch-up angle/force/sine integration. A sustained
  image period>0.25s pauses updates; invalid/stale vision is never consumed.
- Force receipt/source delays and exact producer warming_up, stale_input:*,
  missing_or_invalid_input:*, input_time_skew pause and auto-resume. Other invalid
  force, inference_failed, valid NaN, model/frame/scope errors, overforce, and
  motor/loadcell/tension faults still latch. Paused feedback does not send a
  physical hold; motor may finish the prior target. ID-display-only policy stays.
- Historical warning1792998445.553369077 was preceded by MOTOR/LOADCELL freshness
  FAULT, not a recognition fault. Log `/home/daeyun/.ros/log/robot_control_90070_1792997582192.log`
  lines90/95. Old combined message cannot identify which sensor. New log names
  missing motor/loadcell separately and goal rejection includes actual status.
- Validation: isolated build passed;62 C++ tests passed. Isolated ROS smoke
  suites passed for position(DDS78,102diagnostics), admittance(DDS79,350diagnostics),
  sine(DDS82,114diagnostics/86references), and velocity profiles(DDS79, runtime,
  startup, types/ranges/atomic batch failures). All had0 actuator messages.
  Test build `/tmp/hrm_position_sine_20261026.nRlJ8d`, logs:
  `/tmp/hrm_position_smoke_zi8uapl0`, `/tmp/hrm_admittance_smoke_lzcd4fug`,
  `/tmp/hrm_position_sine_smoke_rqs9sveg`, `/tmp/hrm_motor_profile_smoke_r4y0m7fp`.
- Live read-only audit found controller INACTIVE but motor_output_enabled=true;
  streams healthy at that time. No live commands, gains/modes/gates changed,
  restarts or normal-workspace executable replacement. User must safely stop
  motor output/controller then build robot_control_pkg and restart to deploy.

## Configurable no-load policy + ID visualization-only force control (2026-10-26)

- User requested separate editable load bounds, then explicitly clarified:
  **ID는 표시만, 힘 제어 계속**, including ID9 misclassification during physical
  tip loading. This supersedes older below notes saying all body/unknown IDs
  always block Admittance. Do not reintroduce that unconditional gate.
- New `estimation_pkg/estimation_pkg/id_localization_config.json`: schema1,
  enabledtrue, unloaded_force_ranges_mN initiallyfx±134.326/fy±126.814/fz±192.670.
  Operator subsequently edited it to±200mNallaxes; preservetheeditablefile.
  Compares inferred hrm_base force with published/training sign (NOT FT sensor
  truth, GUI rounded values, or controller-negated force). All axes inclusive
  inside=>UNLOADED/control18; otherwiseLOADED/positiveclassifierwinner.
  Works for masked/class0; rawclass0=0outsidebox usespositivecandidate forcontrol.
- Rawcontact_id/probabilities/continuousforce remain untouched; no forcezeroing.
  status exact `ready_force_ranges` identifies this operating policy. No new
  message fields/wire-schema changes; old bags remain decodable. Disabledconfig
  restoresnativeclass0/maskedUNKNOWN; malformedconfigfails, no silentfallback.
- Nodeparam/launcharg `id_localization_config` picksfile; startup-onlyread and
  runtimeeditsrejected. Packagedvia setup.py toshare root. Subsequent JSON edits
  needrestart, not rebuild under symlink-install. No hysteresis/debounce added.
- IMPORTANT actual default masked KalmanNet/Transformer checkpoint now declares
  force_scope=all_single_contact even though old siblingmetadata stillsays tip.
  Runtimealwaysreads actualbundle. Supporttip/all_single_contact inpolicy/controller;
  unknownscope remainsrejected. No training weights/scalers/featureorder changed.
- Controllerbool `admittance/use_contact_id_gate=false` default explicitlyrequested.
  ID/load/tip_force_valid/bodyflags doNOTgate validFy/Fz in thismode. Realexperiment
  mustrestrict forceto tip; oneXYZoutputcannotseparateID18force frombodyforces.
  Commonvalid/frame/model/window/timestamp/nonfinite/maxforce andmotor/strain
  guards stayintact; no stale-valueholding or automaticmotorenable added.
  SettingtrueoutsideAdmittance restoresoptionalstrictID-gatedeligibility.
- GUI numericrawheatmap retained; UNLOADEDskeletonoverlay hiddenforbothmodes;
  loadedclass0raw0canstilldisplaypositivecandidate. Tipeligibilitytooltip now
  describesID-gateddiagnostic, notactualdefaultcontrolauthorization.
- Validation: purePython policy/config/history118pass; full focused estimator
  suite196tests (22optional archived cases skipped), GUI509functionalpass,
  C++57pass (including19admittance/forcepolicy). PrivateDDS13integrationcases
  covertip/all_single_contact ×masked/class0 ×enabled/disabled +actualMLPbundles.
  ActualdefaultKalmanNet/Transformer withsynthetic26-column30-rowinput produced
  finiteforce/rawID5 andUNLOADED/control18; implementationtest, NOTaccuracy.
- NewprivateDDS79 controller smokePASS16.67s/242position+242admittancediagnostics,
  0actuatorcommands. AllbodyIDs1..17/unknown/unloadedallowcontinuousvalidforce
  withdefaultgatefalse; stale/NaN/model/frame/scope/maxforce/tensionstillfault;
  optionalgate trueandmode-timeparamrejectionverified. Test updated torequire
  explicitisolatedbinary andemptyisolateddomainbeforepublishing syntheticinputs.
- Allthreepackages built under `/tmp/hrm_position_sine_20261026.nRlJ8d` (newJSON
  installsymlink verified). Live GUI/controller/inference detectedrunning;
  NO normalbinaryreplacement, live servicechanges, restarts, or motormovement.
  User muststopmotors/control, rebuild estimation_pkg+gui_py_pkg+robot_control_pkg
  normally, thenrestarttoapply. Earliernormalbuildsection predates thispolicy.
  TestXML/JSON inthat/tmpdir; controllerlog `/tmp/hrm_admittance_smoke_39w49uvi`.
- Operator edited JSONduringwork; preserved theirvalues/commentednotes, added
  whole-line //comment support (inline/blockcommentsstillinvalid). Tests validate
  editablefile semantics withoutlockingtheuser'sthresholds totrainingdefaults.

## Position / Admittance sine reference (2026-10-26)

- User requested m-based sine Start/Stop in Position and Admittance, in addition
  to the existing angle-based Kinematics waveform. One stacked GUI row switches
  with mode, preserving independent values; no extra vertical row/scroll.
- Move Tip Y/Z are absolute hrm_base sine centres even when manual Publish uses
  Relative. Degree and metre Move Tip entries are remembered separately.
  Defaults: Ay=Az=.010m, phi_y=0deg, phi_z=90deg, T=20s. A=0 holds its centre.
- New backend `motion.position_sine.y/z=[centre,amplitude,phase]`,
  `.period_sec`, `.max_speed_m_s=.005` (per-axis reference slew/peak bound).
  New SetBool `/position/sine_motion`, transient String `/position/sine_status`,
  PointStamped `/position/sine_reference` (nominal, hrm_base metres, image stamp).
- Start requires ready Position/Admittance and no other motion. Approach first
  phase smoothly, update only nominal Y/Z on accepted new image-source frames;
  X passive. Admittance then adds its force offset. No per-step PID/filter/origin
  reset, and no gain/force eligibility/motor gate changes.
- Camera gaps freeze phase without catch-up; faults/mode/gate changes stop.
  Stop holds last nominal target with feedback still running, not motor disable
  or emergency stop. Manual goals/other motions/wave edits rejected while active.
- Validation: GUI functional504pass (lint/copyright excluded), C++47pass across
  FK/IK5, positionPD19, IKsine5, CartesianSine9, admittance9. Offscreen1366x768
  Kinematics/Position/Admittance screenshots fit without added right scrolling.
- New `test/position_sine_smoke.py`: isolated DDS82, localhost, motor gate false
  throughout and motor topic additionally remapped. PASS:130syntheticframes,
  113positiondiagnostics,83references,0actuatorcommands. Checks readiness, config
  rejection, source-stamp correspondence, ramp/quadrature/Xpassive, manual-goal
  conflict, stop, image-gapfreeze/resume, mode/faultstop and admittanceoffsetsum.
  Existing position_frame_smoke alsoPASS in DDS78 (107diagnostics,0commands).
  Gate-enable changes were reviewed statically, not exercised live.
- Build/install: isolated `/tmp/hrm_position_sine_20261026.nRlJ8d` and then normal
  workspace `colcon build --symlink-install --packages-select robot_control_pkg
  gui_py_pkg` bothpassed. This normal controller build ALSO includes the earlier
  absolute-position PD velocity-law change; its former isolated-only deployment
  note below is historical. No GUI/real controller/hardware was started/restarted.
  Motor output must be off before the user restarts GUI/controller for real tests.
- Installed normal-workspace binary was also smoke-tested in isolatedDDS82 with
  gatefalse:PASS132syntheticframes/114diagnostics/85references/0actuatorcommands;
  logs `/tmp/hrm_position_sine_smoke_2u_sgp6k`. Test node exited normally.
- Test XML/result JSON in `/tmp/hrm_position_sine_20261026.nRlJ8d`; screenshots
  `/tmp/hrm_position_sine_gui_wfqng6ap/{kinematics,position,admittance}_1366x768.png`.
  Existing compiler reorder/unused-variable warnings remain; no physical tracking
  accuracy/stability is claimed by the synthetic integration tests.

## Contact heatmap on GUI skeleton image (2026-10-26)

- User requested implementation after a read-only feasibility explanation.
  Added GUI-only Contact overlay checkbox (default ON) in the existing image
  controls row. Existing numeric ID heatmap and force routing remain unchanged.
- New custom_interfaces/SegmentImageGeometry on
  `/estimated_segment_image_geometry`: same crop image stamp/frame/dimensions,
  18 blocks of 9 pixel samples plus 18 centers. Coordinates are crop u/v pixels,
  NOT metric XYZ or TF. Contact ID i uses physical segment i, skipping fixed
  os at index0; ID18 covers the final moving segment through the last boundary.
- Estimator retains indices through finite/duplicate XYZ filtering and maps its
  existing raw_s_3d/s_segment/s_segment_center back to original extended skeleton
  pixels. Does not project hardware-length-scaled XYZ. Tiny normalized endpoint
  roundoff is tolerated. Geometry calculation only runs with subscribers and its
  failures cannot suppress angles, normal skeleton, cloud or marker publication.
- GUI shares existing ForceIdPreview with ImagePreviewPanel, no extra model or
  inference subscription. Independent QPainter overlay changes no source RGB,
  published image, record, angle/FK, filter, controller or force eligibility.
  Red/white fixed0..1 intensity uses raw probabilities inside the same ±tol band;
  selected candidate has blue outline/ID. No conditional probability rescaling.
- Geometry must exactly match image frame/stamp/dimensions (receipt age <=0.2s).
  Force stamp is newest selected input, so allow source gap <=0.5s and force
  receipt age <=0.5s rather than exact image equality. Invalid/stale/missing or
  Class0 unloaded/ID0 clears overlay; control fallback ID18 is not tip contact.
  Masked load status remains unknown. Force-only updates, expiry and checkbox
  changes redraw without RGB conversion. Body-monitor OFF retains diagnostics.
- Validation: GUI functional suite 480 tests, estimator focused regression
  92 tests; offscreen layouts including1366x768 unchanged. Four archived RGB/depth
  datasets ×10 frames =40/40 reconstructed/geometries valid; helper median0.116ms,
  maximum0.244ms in this run. Numeric angle/tip outputs unchanged by helper.
  Screenshots show archived images with explicitly SYNTHETIC ID1/9/18 probabilities,
  not learned localization accuracy. Artifacts/script:
  `/tmp/hrm_contact_overlay_20261026.j6sNi4/` (temporary).
- Built custom_interfaces, estimation_pkg, gui_py_pkg successfully. Existing live
  nodes were not restarted; no live ROS service/motor/control command was sent.
  Installed imports and new-message serialization roundtrip checked. Restart GUI
  and Segment estimation safely with motor output OFF to display new geometry.
  This feature itself does not require robot_control changes/rebuild.
- Details: src/gui_py_pkg/README.md contact overlay section and
  src/estimation_pkg/docs/force_id_inference.md. Existing PD rebuild instructions
  below still apply independently of this visualization feature.

## GUI position gains and requested default force/ID bundle (2026-10-26)

- User requested GUI gain adjustment and Masked force=KalmanNet / ID=Transformer
  default, then explicitly chose cable+loadcell+joint-angle inputs.
- ForceModelSelector preselects ONLY the exact existing bundle
  `cable_loadcell_angle/models/masked__force_kalmannet__location_transformer/hrm_bundle.pt`
  on the initial scan. Refresh preserves manual/blank choice; missing default
  never falls back to another feature group. Inference auto-start remains OFF,
  running model not switched; CLI/node default model_path remains blank/explicit.
- Actual bundle loaded/predicted offline on CPU: masked, kalmannet/transformer,
  26 features (cable length4 + loadcell4 + relative_angle18), 30-sample window,
  force N/hrm_base and probability vector19 (p0=0, no class0/no-load probability).
  all_single_contact scope does not bypass existing tip-force eligibility guards.
- Compact PositionPIDPanel below IK sine uses GetParameters and atomic six-gain
  SetParametersAtomically on /robot_control. Pan(Z),Tilt(Y) separate P/I/D.
  I disabled/fixed0 by existing PD policy; successful Read needed before Apply.
  No auto-write on launch/reconnect, no motor enable/mode/goal commands, no
  persisted gain override. Controller restart resets startup gains; explicit Read
  refreshes actual values. Timeouts/rejections are not displayed as applied.
- GUI/readme and inference docs describe defaults and units. The 2026-10-26
  rate-PD controller uses P rad/(m*s), D rad/m; GUI cannot change a running old
  binary's control law. Previous controller rebuild/restart instructions still apply.
- Verified 447 GUI functional tests (style/copyright checks excluded), including
  19 fake parameter-service PID tests. Offscreen layout fits 1366x768, 1500x900,
  1920x1080 with aligned-record settings on/off and all three image previews;
  existing right-side spacing/heights were reduced to accommodate the PD rows.
- `colcon build --symlink-install --packages-select gui_py_pkg` succeeded; installed
  import and exact default-bundle discovery verified. Existing GUI/controller/model
  processes were NOT restarted; no live ROS gain/mode/motor requests were sent.
  Restart GUI to load widgets; controller source still needs its separate rebuild
  from the preceding PD task before using the new control law.
- GUI tests/screenshots: `/tmp/hrm_gui_controls_20261026.vse1FT/` (temporary).

## Absolute Y/Z position PD correction (2026-10-26)

- User confirmed Absolute Z=0.010m, observed FK near3mm. Previous read-only live
  probe: goalZ10mm/actualZ3.158mm, ACTIVE ~30Hz; no GUI m/mm scaling error.
  Main cause: fixed `entry_angle + PD` offset, Kp5rad/m/I0, leaving large error.
- Restored scalar PID **error law** through `compute_output`: e, I*integral(e dt),
  D*filtered((e-e_previous)/dt). Position keeps I=0 (no stored error integral).
  D-on-measurement changed to D-on-error; tau parameter remains functional.
- Output explicitly angular velocity rad/s; update current command by
  `clamp(u,±speed)*min(image_dt,.25)`, then clamp angle. No fixed entry reference,
  no frame-rate-dependent full-output addition. q-entry initializes only;
  encoder/cable entry offsets MUST remain for bumpless motor conversion.
- Numerical defaults unchanged P5/I0/D.05 but units now P rad/(m*s),D rad/m.
  Keep±60deg,5deg/s,source-frame pacing,sensor/tension gates,camera-gap reset.
  Scalar PID optional tau defaults0 for existing 3-arg callers; vector PID unchanged.
  Use rad/torad for the local IK degree conversion to avoid float round-trip drift.
- This changes the shared PositionController inner loop for Admittance as well;
  force routing/admittance law/GUI/kinematic-angle sine generator are unchanged.
- Package built in isolated `/tmp/hrm_position_pd_20261026.n76Nlu`, NOT normal
  workspace install. Running hardware controller not restarted/replaced; no motor
  commands or runtime parameter changes. User must motor-output OFF/stop old node,
  rebuild and restart explicitly before hardware verification. Gains remain slow
  bring-up values; ideal FK tests cannot certify physical stability/tracking.
- Current controller docs: `docs/CONTROL_READINESS.md` top section supersedes
  September fixed-entry PD sections; `src/robot_control_pkg/README.md` updated.
- Verified38C++tests:19PD+5FK/IK+5sine+9admittance/eligibility. IdealproductionFK
  +/-10mmsteps fromzero/nonzeroentry,dualaxis,10/30/50Hz,constantdisturbance,
  small/slowposition sine pass. NOT realpositionaccuracy or fasttracking proof.
- Updated position_frame_smoke.py for ratePD (isolatesP withD0) and optional
  HRM_CONTROL_TEST_EXECUTABLE. Domain78/localhost/disabledoutput/remappedmotor
  integration passed105diagnostics,0actuator messages; synthetic sensors only.
  Logs `/tmp/hrm_position_smoke_nmnv4scz/control.log`. No actualhardware changes.

## Explicit base/tip-near perspective follow-up and paper assets (2026-10-24)

- Sibling Segmentation repo: `runs/perspective_pose_followup_20261024/report.html`.
  User requested explicit base-large/tip-small and reverse perspective augmentation,
  plus organized image/pose evidence for the paper. No ROS/controller changes or motor commands.
- `feasibility_augmentation_v3.py` uses a source-base-to-tip-aligned virtual textured
  planar card, pinhole tilt 25/45/60 degrees, both near-end directions. These are NOT
  real joint angles or novel articulated 3D/depth views. Changed RGB is never paired
  with unchanged source depth. Some original-background composites have inpainting seams.
- New V3 U-Net: same V2 train 1600 / val 96 records, 16 epochs, seed173, batch8,
  size384, optimizer and validation conditions. Fresh ImageNet initialization, not V2
  fine-tuning. Single-seed development comparison, not blind final evaluation.
- Evaluation: 128 sources x11 conditions x3 methods =4224 rows, zero runtime errors.
  V3 plain perspective Dice 97.78–98.34%, background+placement+3-gap 96.18–96.52%.
  Improvements over V2 are small (plain max +0.44 pp); clean falls98.09→97.93%.
  OpenCV remains higher on simple perspective pseudo-labels; learned advantage is
  composite background/occlusion robustness, not universal superiority.
- All1408 augmented RGB/visible-label/occlusion cases and4224 predicted masks are
  saved under `segmentation_evaluation/assets/<condition>/<source>/` with timestamp,
  source SHA256, H, magnification and depth-incompatibility metadata. Per-case
  provenance.json, cases.json and figure_sources.json map paper figures to originals.
  figure_sources relative asset paths use asset_path_base='segmentation_evaluation'.
- Strong near-base taper can create spurious medial-skeleton endpoint branches.
  V3 synthetic occluder placement therefore follows the SOURCE skeleton transformed
  by the SAME H; this guide is only label-generation metadata, never a network input.
  Frozen V3 preflight2048 cases passed; full suite122 tests passed. Independent PNG
  audit recomputed54912 metrics from all4224 predictions with zero discrepancies.
- `paper_pose_summary.{html,csv,json}` is a SEPARATE re-analysis of V2 original RGB-D
  occlusion results, not V3 warped-image poses. It includes per-condition availability,
  mean/median/p95/max, <=1mm fractions, q18 wrapped RMSE and SO(3) rotation difference.
  `paper_pose_source_index.json` links104 original RGB/depth/K/source metadata/NPZ.
  In3-gap30%: mean FK delta0.413/0.554mm and <=1mm95.0%/90.6% (straight/bent).
  Across all624 occluded frame-condition pairs: max FK delta2.280/3.716mm.
  These are own-clean estimate differences, NOT actual GT accuracy or all-frame1mm
  guarantees. Known synthetic occluder depth exclusion and physical/manual-GT caveats remain.

## Segmentation V2 and paired angle/FK occlusion sensitivity (2026-10-24)

- Follow-up work lives in the sibling Segmentation repo; README and
  `runs/occlusion_pose_20261024/report.html` are the entry points. No main ROS
  estimator/controller changes or motor commands. Captured80fresh alignedRGB-D
  pairs with new black background using camera-only D405CUDA launch; stopped
  only owned camera process. Stored camera/ROI/K/source timestamps.
- New V2 U-Net1600trainingframes/16epochs, validation96x3. Augmentation includes
  single2/10/20/30% and3-stripe15/30% centerline occlusion, white/gray/black/color
  materials, backgrounds, hard-negativebrightobjects, scale/rotation/translation/
  planarprojectivehomography. Preserve visible-only labels. Not true3Dviewpoint
  synthesis or wide-angle rendering. Source data/previousmodels untouched.
- Segmentation128frames x10conditions x3methods. Bg+projective+3gaps Dice:
  OpenCV20.33%,V1 84.49%,V2 96.03%; multi3_30:31.69/95.62/96.50. V2 slightly
  regresses some simplecases; not uniformlybetter. Same pseudo-label limitations;
  archive was seen in pilot1, so this is development stress testing, not blind.
- Offline RGB-D evaluation40fresh+64archived frames x7conditions x3methods.
  Archive45deg1 RGB/depth filename indices are shifted; exact source_time_ns join
  is mandatory. Occlusionmap invalidatesdepthonly; never oracleclip prediction
  or reuse hidden depth. This is missing-depth simulation, not automatic rejection
  of valid real-occluderdepth. RGBprojectivecases excluded fromRGB-D evaluation.
- feasibility_shape.py reuses exact productionquartic/reconstruct/projection via
  AST, no originalmoduleimports. 19geometriclinks/18tilt-first localZjoints;
  PythonFK verified against compiled original surgical_tool.cpp methods/Eigen.
  Newfront-end: all visiblepieces, graph skeletonpaths, nearestendpointordering,
  mask-awaredepthmedian, chord approximationacrossgaps; no2Dextrapolation.
  Kalman/baseaverageOFF, sameframeowncleanbasecalibrationfixed. Metrics mean
  per-frameq18wrappedRMSE,FKpositiondistance/SO3rotation difference, notGTerror.
- Initial pose_evaluation preserved: minpath8 wronglydiscards shortvisiblebase
  stubs aftermultiocclusion. Optionalminpath1/singleton preservesactualobserved
  pixels, nohiddenpointinsertion. Finalvariant pose_evaluation_short_fragments
  recordedseparately with sameweights/cases. This is explicitposthocfrontend
  development, not an independentvalidation. Failuresalwaysremainindenominator.
- Finalmin1V2 outputs728/728cases(104framesx7), notGTaccuracy100%. multi3_30
  live40frames meanq18RMSE1.265deg/FKdelta0.413mm/rotation3.305deg, dynamic64:
  1.477deg/0.554mm/3.603deg. Tipdelta p95 .804/1.390mm. Initialmin8V2availability
  15/40 and31/64 recovers40/40 and64/64. No assertionthat actualerror<1mm.
  Finalallmethods1675/2184valid;OpenCV492fail,V1 17fail,V2 0fail.90testsPASS.

## Occlusion segmentation feasibility — separate repository (2026-10-24)

- User authorized augmentation/training/comparison and camera-only capture to
  examine visible-body masking under occlusion for reviewer response. Work is in
  `/home/daeyun/Hyper-Redundant-Manipulator-Segmentation`; see its README and
  `runs/feasibility_20261024/report.html`. Production ROS estimation/control code
  is NOT switched to learned segmentation. No motors or controller were started.
- Trained basic and robust-augmentation ResNet18 U-Nets: 1,200 frames from two
  sessions, 12 epochs each; validation 64 frames with fixed clean and occlusion+
  background conditions. Current-manifest held-out candidate 45deg_1 supplied
  96 test frames, 10 conditions, 5 backends (4,800 predictions, zero exceptions).
  Historical development exposure unknown; masks are OpenCV pseudo-labels, NOT
  independent manual GT or clinical validation. All original datasets preserved.
- Middle 20% synthetic occlusion mean visible-mask Dice: OpenCV71.90%, basic
  U-Net91.20%, robust U-Net97.76%, robust+MobileSAM97.52%. Background+20%:47.18%,
  87.03%,97.78%,98.69%. Full-ROI MobileSAM often picks background (middle-occlusion
  Dice0%); weak target prompt, not evidence that SAM generally cannot work.
  U-Net~5ms, hybrid~49ms on RTX3060; no camera/ROS/3D time included.
- Occluders remove covered pixels from labels; hidden geometry is NOT invented.
  Original OpenCV largest-component selection often discards one visible half.
  Any future production integration must retain multiple visible pieces and
  separately constrain depth sampling to observed-body mask pixels; do not feed
  imagined/amodal pixels or arbitrary neighboring occluder depths into 3D fit.
- Captured20 paired color/depth crops with existing D405 CUDA camera-only script,
  same source timestamps, then stopped only that newly launched camera process.
  Straight/unoccluded live images plus synthetic10/20% conditions:300 inferences,
  zero exceptions; no GT accuracy score. Robust U-Net retains body but can detect
  a bright corner object; SAM refinement can amplify this into background. These
  failure examples are retained. Actual physical-occlusion test remains needed.
- Additional offline depth diagnostic uses clean live masks only, in-mask5x5
  median (>=3 valid samples), crop intrinsics once, camera-frame XYZ; no gap
  interpolation/curve/angle/FK/ROS output. False-positive mask can still admit
  wrong depth. RGB augmentation must never reuse unchanged real depth geometry.
- Depth preview actually exported20cleanpairs,3978candidates→3881finitepoints;
  background false positives produced depths0.113–0.663m, so these are NOT all
  verified HRM points. Retained as a failure example. Strict source/condition/
  timestamp/RGB provenance guard rejects synthetic masks.54CPUtests passed.
- Local isolated segmentation `.venv` reuses HRM_env torch2.7.1+cu126 via .pth;
  HRM_env/system ROS dependencies unchanged. New torchvision0.22.1+cu126 and
  segmentation/SAM packages local only. No large dataset copies. Models/plots/
  source snapshots are ignored artifacts; copy them explicitly to another PC.

## Shape-dataset pre-skeleton binary PNG export (2026-10-21)

- User requested the final segmented/smoothed binary mask immediately before
  skeletonization, from four September27 shape datasets. Actual parent directory:
  `record/20260927_HRM_shape_estimation_apriltag` (not the shortened name in chat).
  Read-only SQLite inspection of all four bag_0.db3 files found36 topics each,
  **no Image/body-binary/skeleton image topic**. Live code can publish
  `/estimated_segment_body_binary_image`, but it was not recorded in these bags.
- Reprocessed every existing `images/hrm_crop/*.png` with CURRENT production RBSC
  segmentation: unchanged HSV exclusions → grayscale threshold120 → largest
  8-connected component → closing15 → filled external contours → Gaussian15 →
  threshold127. Exact requested output is `RBSC.body_image`, not binary_image.
  Added optional `postprocess(..., segmentation_only=True)` to return at this
  stage without skeleton/curve/depth/calibration. Default live path unchanged.
- New offline script: `scripts/export_hrm_crop_segmentation.py DATASET_ROOT`;
  source ROS/workspace and use HRM_env Python, MPLBACKEND=Agg. `--inspect-only`
  audits without writes. Requires new destinations and refuses overwrite.
  Uses archived PNG as BGR (archive already converted original ROS RGB); no new
  crop, normalization, depth processing or image resizing. No ROS playback,
  camera launch or motor command. Current-code reproduction does not claim exact
  historical online frames (those were also gated by successful3D reconstruction).
- Created `<session>/images/hrm_crop_segmentation/` for all four datasets:
  35deg_1:2241; 35deg_2:1529; 45deg_1:1670; 45deg_2:1619. **7059 saved,0 failed**.
  Lossless mono8 PNG, body255/background0, same filenames/dimensions as crop RGB.
  Combined PNG payload10,219,662 bytes (~9.75MiB); index/manifest small extra.
  Original RGB/depth, bags, recording settings and numerical CSVs untouched.
- Each output folder has index.csv retaining ALL source index columns/timestamps,
  plus status, output filename, foreground area, PNG size and source/output SHA256.
  manifest.json records algorithm/source hash, checked bag inventory and counts.
  Original index input_encoding/payload_bytes still describe RGB; mask_encoding
  and mask_png_bytes describe derived masks. Failed frames would be explicit,
  never silently reuse the last successful mask or invent all-black success.
- Validation: all7059 PNGs reread, verified same size, uint8{0,255}, hashes/areas,
  filename/index correspondence and unchanged source-image hashes. Sample RGB
  and mask visually checked. 13 new segmentation tests +11 existing realtime
  pipeline tests passed (24 total): independent stage parity, BGR/RGB, no later
  stages, unchanged default continuation, invalid-frame/recovery behavior.

## Learned force connected to Y/Z admittance (2026-10-15)

- User requested `/force_id_estimation` → Y/Z position/admittance integration and
  target-force test readiness. Implemented software path; **no physical motor
  movement, production controller launch or powered stability validation**.
  This supersedes the historical deferred-control notes in the 10/12 entries.
- New force_feedback.hpp validates the combined ForceIdEstimate: hrm_base,
  finite XYZ N, class0 tip-scope, consistent raw/load/control IDs and tip flags,
  complete window, source/publication + receipt freshness. Loaded ID18 or unloaded
  raw ID0/control ID18 eligible; unloaded force retained. Body ID1..17 ineligible
  with monitor OFF or ON; masked remains inference-only pending a load detector.
  Legacy Vector3 force no longer refreshes/authorizes admittance (retained only
  for legacy dynamics, which this task did not validate or rewrite).
- Apply documented training-sign negation once (`admittance/force_sign=-1`), no
  axis swap or N conversion. F_env/target mean external force ON HRM in hrm_base.
  X remains observable but passive. Equation on Y/Z is Mδ''+Bδ'+Kδ=F_env−F_target;
  reference Y/Z + δ feeds existing PD (Y→tilt, Z→pan), then cable IK/motor preview.
  The existing motor-output gate still defaults false; no auto-enable anywhere.
- New target_force_y_N/target_force_z_N parameters default zero and may be edited
  live. Defaults M=.05 kg, B=1 N·s/m, K=20 N/m; added initial offset±5 mm per axis,
  velocity±2 mm/s, estimated XYZ/target YZ force-norm limit2 N, timeout.25 s.
  Values are unvalidated bring-up defaults, not hardware safety ratings. Gain/
  sign/limit edits require leaving mode4; complete parameter batch validation.
- Replaced singular 6x6 inverse/explicit Euler with two-axis implicit Euler and
  bounded state, no hidden velocity windup. Reset at fresh mode/gate entry with
  PD/encoder origins. Source-image paced; long camera gaps freeze integrator,
  zero velocity and skip catch-up, preserving existing camera auto-resume.
  Force-model history loss during camera gaps still causes a force fault.
- Existing feedback-fault behavior reused: WAITING before readiness; afterwards
  stale/invalid/body/unknown/overlimit force/model change triggers current-encoder
  software hold if fresh and latches until mode exit/re-entry/gate transition.
  No previous-force reuse or synthetic0N and no body-evasion commands. Async
  question asked whether user prefers this manual recovery or automatic recovery;
  no response received at implementation time. Do not claim explicit approval of
  that policy. Motor/loadcell/tension guards unchanged. Driver watchdog/E-stop
  remain essential; stale motor feedback prevents computing a fresh hold.
- Updated GUI explanatory label (no new controls), inference/controller docs and
  message comments to avoid claiming that inference has no controller consumer.
  Target commands/test steps live in src/robot_control_pkg/README.md.
- Validation: robot_control build passed (existing warnings); all31 C++ tests
  passed (5 FK/IK,12 PD,5 sine,9 new admittance/eligibility),18 GUI force-preview
  tests passed. Private-domain position smoke passed. Admittance smoke checks
  legacy rejection, N/sign/X-passivity, two-axis motor previews, target changes,
  atomic rejection, camera gap/recovery, body/unknown/stale/NaN/model/force/tension
  faults, reset, and asserts ZERO actuator messages. No learned-model physical
  accuracy or closed-loop plant test. Existing inference parity tests remain
  documented in the earlier section.
- Remaining: verify hardware PD directions, force sign/calibration and model lag;
  choose recovery policy and masked load detector; tune actual contact dynamics.
  With K>0 steady force error=Kδ, so target-only operation does not ensure exact
  force tracking. K=0 removes spring bias but does not establish stability or
  contact in free space. Axial X and Cartesian cross-coupling remain uncontrolled.

## Force inference tip/TF input support (2026-10-12, follow-up)

- User added input-combination bundles and requested fk_tip_tf / estimated_tip
  support plus a simple explanation of automatic arrays. Model feature_columns
  supplies the order; feature_spec maps each name to live fields, and
  FeatureHistory.sample builds the numeric row in that exact order. No per-model
  hardcoded array is necessary, but new feature kinds require an adapter.
- Added PointStamped sources `/estimated_tip_position` and
  `/kinematics/fk_tip_position`; TFMessage `/tf` filtering direct hrm_base parent
  and children `hrm_fk_tip` / `estimated_segment_center_19`.
  `fk_tip_tf` includes m XYZ + xyzw. Estimated-pose models combine final boundary
  XYZ (`estimated_tip`) and last-center quaternion (`estimated_tip_orientation`),
  matching recorded summary columns; never substitute the center's translation.
  Original quaternion values/signs retained; optional Euler degree fields follow
  recorder formulas. Wrong frame, zero/nonfinite quaternion or invalid XYZ fails.
- FeatureHistory now tracks TF children separately from physical topics so a
  live unrelated TF broadcaster cannot conceal stale/missing tip data. One /tf
  subscription with depth 100, all other subscriptions keep previous depth 1.
  Each transform's source stamp is preserved. No TF lookup/interpolation, frame
  remapping, controller changes, hardware launches or motor commands.
- Current source models: six groups x 242 = 1,452 bundles. Groups and per-row
  dimensions: cable_loadcell 8; cable_angle/loadcell_angle 22;
  cable_loadcell_angle 26; cable_loadcell_fk_tip_tf/estimated_pose 15.
  All 1,452 input mappings + strict restores + finite inference passed. Existing
  archived numerical parity tests remain cable/loadcell-only. Unit/runtime/
  shutdown/pose tests: 147 passed; isolated DDS-domain synthetic ROS tests with
  real cable/FK-pose/estimated-pose class0 + masked models: 3 passed, including
  ordered input values, model switches and stale recovery. Not physical accuracy.
- Updated force_id_inputs.py, force_id_estimation.py, package.xml (tf2_msgs),
  tests and docs/force_id_inference.md. Rebuilt estimation_pkg with symlink install
  to install the newly added model groups without copying NPZ archives.

## Optional force/contact inference implemented (2026-10-12)

- Read user-provided `docs/CHATGPT_WORK.md` section 97. Current scope is learned
  inference + GUI diagnostics, **not motor/admittance integration**. No motor
  command or camera was started for these tests. User freed disk; ~72 GiB free
  at handoff. Existing dataset edits/deleted legacy H5 files and unrelated GUI
  serial-port changes were preserved.
- New `estimation_pkg/force_id_estimation.py` node `/force_id_estimator` and
  standalone `force_id_estimation.launch.py`. New combined topic
  `/force_id_estimation` (`custom_interfaces/msg/ForceIdEstimate`) contains XYZ
  force, raw/control ID, original 19 probabilities, readiness, load state,
  model/window/timing metadata and body/tip eligibility flags.
  It never publishes legacy `/estimated_external_force` or actuator commands.
- Models discovered recursively under `src/estimation_pkg/models/**/hrm_bundle.pt`.
  Current 242 schema-2 `independent_hrm` bundles: class0/masked, 11 force x 11
  location architectures. `force_id_runtime.py` / `force_id_models.py` restore
  strict state dictionaries with weights_only loading, independently normalize
  each branch, and inverse-scale force. `.npz` is archived validation output,
  not the scaler; scalers are in `.pt`. Some copied experiment JSON/YAML files
  are empty and unused. Install only `.pt`, not large offline NPZ files.
- Bundle feature order/window is authoritative. Current inputs: four wire
  lengths (mm, `/wire_length`) + four unchanged loadcell tensions
  (`/loadcell_state`); window 30. No camera, F/T ground truth or unused angles
  required. Future angle features use odd tilt/even pan, radians. Unknown or
  target/label features fail explicitly. `force_id_inputs.py` checks finite
  values, new source messages, age/skew/gaps and clears history on invalid data.
  Default timer 30 Hz, timeout .25 s, max skew .1 s, max window gap .15 s;
  30 fresh rows required after startup/recovery. Headerless wire uses receipt
  time. This is not exact camera-stamp alignment of the training dataset.
- Forces are N / hrm_base, **training sign unchanged**. Future control negation
  (-1) deliberately deferred. Class0 raw ID0 gives UNLOADED/control ID18 without
  replacing force or probabilities. Masked has no load classifier: UNKNOWN,
  control ID -1, tip_force_valid false; no invented 150/200 mN threshold.
  Loaded raw ID1..17 is not valid tip force (current force models are tip-only).
  Invalid output uses NaN/-1, not a fabricated zero-force observation.
- GUI Packages has optional **Force / contact**, automatic start OFF. Explicit
  model selection + Refresh; stop component before selecting another bundle.
  Body monitor and +/-ID can update via estimator parameters. Heatmap uses raw
  ID1..18 probabilities and active-ID argmax +/- tolerance, never renormalizes
  or treats band probability as accuracy. Fx_pred/Fy_pred/Fz_pred display mN,
  base-frame labels; measured F/T remains sensor-frame. Unified stream invalid
  or stale never silently falls back to the legacy force topic. Body setting
  is diagnostic/future-policy only, not an implemented stop/hold controller.
- Launch uses HRM_INFERENCE_PYTHON, else ~/HRM_env/bin/python3 when available,
  else current Python. Here ROS system Python lacks Torch; HRM_env has it.
  CPU / one BLAS thread by default. GUI does not import Torch. Model-load
  failures show status; invalid dynamic selection keeps the previous model.
- Verified: custom_interfaces + estimation_pkg + gui_py_pkg symlink builds;
  all 242 bundles strict restore/finite inference; 22 architecture/mode pairs
  compared to archived NPZ outputs (three windows each, atol 2e-6); 113 input/
  runtime tests, five shutdown tests, 422 GUI tests, isolated DDS-domain ROS
  integration test (warm-up, stale/recovery, model switch, body/ID parameters,
  no legacy-force/motor publishers). Standalone launch loaded the real bundle
  and exited cleanly on SIGINT. Synthetic tests do not validate physical force
  accuracy. Transformer requires tanh-approximate GELU per archived parity.
- Next control discussion: current robot_control legacy force callback copies
  only x/y; adopt combined XYZ/frame/validity, apply agreed sign once and use
  **Y/Z admittance only**, X observable. Agree masked load detection, body
  behavior and invalid/stale force policy before enabling any motor response.
- Usage/contract/tests: `src/estimation_pkg/docs/force_id_inference.md`;
  GUI details in `src/gui_py_pkg/README.md`. Training code remains separate.

## September 26 sine dataset pose refresh (2026-10-11)

- Later user request: collected all 40 refreshed CSVs under dataset root
  `summaries_sine_35deg_both/`, named `summary_<session-name>.csv`. Initially hard
  links, then converted to INDEPENDENT COPIES at the user's explicit request.
  Collection has 238,401 rows across its files and adds 859,667,824 CSV bytes.
  Original per-session files retained; source/copy SHA256 matches and distinct
  inodes verified. `collection_manifest.json` records source paths, row counts,
  hashes and independent-copy semantics. Editing copies does not change originals;
  future source refreshes do not update copies automatically. Do not double-count
  originals and collection in training.
- Refreshed `record/20260926 외력 추정 데이터 1/*/csv/summary.csv` in place:
  40 sessions, 238,401 actual existing rows, unchanged 268 columns. Only the four
  tip groups changed using production `summary_csv.py` unbounded nearest logic.
  All other cells, manual labels/force edits, timestamp formatting and row order
  are identical to the old CSVs. No deleted rows were restored.
- Six CSVs have spreadsheet-rounded scientific-notation timestamps. Matched them
  to exact original angle stamps using rounding bounds, receive times and elapsed
  times; preserved displayed cells while tip time differences use exact anchors.
  Four old schemas overstated rows by 115 total (ID15/16/17/18 right); updated
  schema row counts to actual CSV counts, without adding/removing any data rows.
- Source tip/TF exports have zero decode errors; message counts agree with bag
  metadata and actual CSV row counts agree with export manifests. Reused existing
  topic CSVs instead of redundantly exporting whole bags. Bags and topic CSVs
  unchanged. No ROS replay, hardware node launch or motor commands.
- Blank counts before -> after: estimated_tip 94 -> 0; fk_tip 108 -> 0;
  estimated_tip_orientation 95,497 -> 0; fk_tip_tf 92,527 -> 0. Largest absolute
  match gaps respectively 66.7 ms, 1.968 s, 2.767 s and 5.300 s. These are recorded
  samples associated without age limits, not interpolated/contemporaneous truth.
- Backups: dataset-root `summary_refresh_20261011/previous_metadata/<session>/csv/`
  contains `summary.csv.gz`, original schema/manifest, and replacement hashes.
  All 40 old compressed and new CSV checksums verified. Entire report/backup folder
  ~307.5 MB (compressed CSVs 303.3 MB); no separate latest-summary collection.
  Refreshed CSVs grew ~40.5 MB total; free space after work ~4.7 GiB.
- Report: dataset-root `summary_refresh_20261011/report.json`. Independent bisect
  validation checked all 953,604 tip-group matches; 480 first/middle/last poses
  decoded directly from read-only SQLite bags agree. 91 summary/startup tests
  passed with ROS imports enabled. No staging folders remain.
- One-off task helpers: `/tmp/refresh_hrm_20260926_poses.py` and
  `/tmp/verify_hrm_20260926_poses.py`. Production bag decoder is
  `src/record_pkg/record_pkg/export_csv.py`; summary joining/conversion is
  `src/record_pkg/record_pkg/summary_csv.py`. The one-off refresh deliberately
  preserves current summary rows/non-tip cells instead of regenerating those.

## Recorded datasets refreshed in place (2026-10-11)

- Collected independent copies of all 18 refreshed main summaries under
  summaries_static_pan_-30to+30_tilt_-30to+30_interval-15deg/ in the dataset root.
  Names: summary_static_pan_-30to+30_tilt_-30to+30_interval-15deg_seg-id-1.csv
  through -18.csv (no right suffix). All hashes verified against refreshed originals.
  collection_manifest.json retains source mapping, row counts, hashes and ID1's
  22 trial boundaries. Total 2.708 GB; original files unchanged. Do not double-count
  copies and original/backup summaries in training. Free disk now about 2.68 GiB.
- Updated all 18 main summaries under record/20260928 외력 추정 데이터들/:
  759,945 rows, unchanged 268 columns. Tip four-group unbounded nearest policy;
  every non-tip cell (including force, angles, labels and timestamps) preserved.
- ID1's 22 originals in seperated_folders were refreshed individually, then
  remerged; no matching across trial boundaries. 39 source summaries + 1 merged.
- All previous summaries retained under summary_refresh_20261011/previous_metadata/
  (about 2.72 GB). Initial eight were replaced before user freed space; their old
  copies were reconstructed and verified byte-for-byte against recorded SHA256.
  All 40 old and 40 new checksums independently verified. Bag/topic CSVs untouched.
- ID10 had rounded scientific-notation timestamps and aligned forces absent from
  source topic CSVs. Preserved existing non-tip cells instead of overwriting them
  with regenerated data; row correspondence checked against source time/elapsed/ID.
- Main-summary blanks now: estimated_tip 170->0; estimated_tip_orientation
  149285->0; fk_tip 115->0; fk_tip_tf 151824->0. Time-gap caveat: max orientation
  difference 3.200 s / FK TF 3.834 s. Filled does not mean contemporaneous truth.
- Report/checksums/per-session time-gap stats: summary_refresh_20261011/report.json.
  Training must exclude backup folders and ID1 separated originals when using its
  merged summary to avoid duplicate samples. Final free space about 5.2 GiB.

## Summary tip/pose nearest-time matching (2026-10-11)

- Latest user request removed the 25 ms bound for estimated_tip, fk_tip,
  fk_tip_tf and estimated_tip_orientation. All four now use UNBOUNDED nearest
  source time (past or future, exact preferred, past wins ties), superseding the
  earlier same-day bounded change. No change to sensor/tag tolerances (25 ms).
  Source stamp and signed time_difference_ms remain truthful. Missing/invalid
  streams stay blank, but distant tips may be reused across gaps/end of stream;
  inspect age in training. Schema groups matching=nearest_unbounded and
  max_time_difference_ms=null. CLI tolerance option only controls sensors/tags.
- Existing 268 columns, angle anchors, startup policy, sensor matching and tag
  detection guard unchanged. Bag and per-topic export unchanged; existing datasets
  NOT regenerated or overwritten. Old summary can be rebuilt from complete topic
  CSVs; actual messages missing from the recording cannot be recreated by matching.
- 458 recorder functional tests passed, including all four tip groups outside
  25 ms, multi-second gaps, no limit even at zero sensor tolerance, missing streams,
  exact/nearest selection, TF child filtering and signed provenance. Symlink imports
  updated source; no build needed. No hardware/ROS nodes started or restarted.

## ID2 interrupted export recovered (2026-09-28)

- Session under record/20260928 외력 추정 데이터들/
  static_pan_-30to+30_tilt_-30to+30_interval-15deg_seg-id-2_right now has
  csv/summary.csv: 20,057 rows, 268 columns, all contact_segment_id=2.
- Re-exported all 1,455,961 bag messages read-only; SQLite quick_check OK,
  zero topic decode errors, receive-time integrity complete. Original interrupted
  csv preserved as csv_interrupted_backup_20260928, bag unchanged.
- Startup policy skipped 26 leading rows; 18 later rows have incomplete core
  matching and remain as blanks (not fabricated/removed). See RECOVERY_20260928.json.
- Original postprocess.json/export.log remain interrupted-export evidence;
  recovered csv/manifest.json is authoritative for the completed recovery.

## Sensor numeric colours (2026-09-28)

- GUI measured F/T six axes and loadcell four channels: abs(value)<=1000 black,
  1000<abs(value)<1500 orange, abs(value)>=1500 red, in existing display units.
  Negative loads included; original signed text/torque formatting unchanged.
- Display only: no sensor, recording, prediction, plot or motor policy changes.
  Missing/nonfinite values clear colour; style updates only on category changes.
- Headless GUI/layout tests: 36 passed including exact thresholds, negative
  values, reset and missing loadcell channels. No production nodes restarted.

## Segment ID 1 summary consolidation (2026-09-27)

- Created record/merged_seg-id-1/csv/summary.csv from 22 completed top-level
  static_*seg-id-1_{left,right} summaries: 20,781 data rows, unchanged 268 columns.
  Ordered sessions chronologically; every original cell, blank and timestamp kept.
  All manual labels are 1. Original sessions untouched; no bag/image duplication.
- merge_manifest.json and source_intervals.csv contain source hashes and 0-based
  data-row [start, stop) ranges. source_metadata/ preserves each source's configs
  and CSV schema/manifest. Use these boundaries for temporal windows/splits;
  original elapsed_s resets between sessions. Do not train on both originals and
  their merged copy. Full output cell equality and unchanged source hashes verified.
- Excluded static_pan_-30to+30_tilt_-30to+30_interval-15deg_seg-id-1_right:
  no summary, internal manifest contact_segment_id=2, interrupted export after
  folder rename (old path missing in export.log). No recovery/relabeling performed.

## Optional crop RGB/depth recording (2026-09-27)

- GUI Recording row has Save crop RGB/depth (default checked, preserves behavior).
  It sends save_images atomically with alignment/contact ID before Record ACK;
  entire settings panel locks through recording/flush/export and reflects frozen
  external-session status, without overwriting next-session edits while idle.
- RecordNode save_images bool defaults true; OFF gates both configured image/depth
  archives and their live requirements, persists effective enabled=false config
  and snapshot.save_images, so exporter audits intentionally absent files disabled.
  Numeric bag/topics/CSV/schema/metadata, GUI previews and motor behavior unchanged.
  ON restores configured archive defaults. Existing sessions remain untouched.
- Scope is crop file archives; default bag is numeric-only. Custom user profiles
  that explicitly bag images retain their explicit topic selection.
- Restart GUI and idle recorder after update, never during capture/export.
- Validation: 822 GUI/recorder functional tests and both symlink builds passed.
  Isolated domain184 synthetic ON/OFF/ON Record/Stop/export roundtrip passed:
  OFF has no images directory, image/depth audits disabled, numeric bag and
  unchanged268-column summary complete; ON restored both archives and previous
  image indexes stayed unchanged. Artifacts /tmp/hrm_crop_depth_record__25uz_6p.
  No production nodes, cameras or actuators started/restarted.

## GUI predicted-force preview (2026-09-27)

- Requested F/T layout: fx/tx, fy/ty, fz/tz paired half-width, followed by
  fx_pred/fy_pred/fz_pred. Upper plot measured solid + prediction dashed XYZ;
  same-axis colours, fixed legend. Lower torque plot retained.
- Read-only `/estimated_external_force` Vector3 subscription, BEST_EFFORT depth1.
  No model, publisher, motor action or recorder changes. Missing/stale/invalid
  predictions are distinguished from zero and plotted as NaN gaps (timeout0.5s).
- Display remains sensor axes/mN. Startup parameters predicted_force_topic,
  predicted_force_unit (mN/N), predicted_force_sensor_axes (signed permutation
  sensor-from-input), predicted_force_timeout_sec. Default assumes sensor-axis/mN
  input; future hrm_base output MUST explicitly use the inverse training mapping.
  Independent of CSV alignment UI. Headerless Vector3 cannot validate source age
  or frame; graph is live preview, not synchronized error measurement.
- Preserve the user's pre-existing Start/Stop button-connect ordering edit.
- Verification: 383 GUI functional tests and gui_py_pkg symlink build passed.
  Domain186/remapped-topic synthetic Vector3 tested RELIABLE/BEST_EFFORT and stale
  recovery path; zero motor messages. Headless layout checked at1366/1500/1920 widths.
  No production GUI/hardware nodes started or restarted; next GUI start loads code.
- No-load analysis handoff from 2026-09-26 is saved under
  docs/analysis/aidin_ft_no_load_validation_1/{README.md,analysis.json,force_noise.png}.
  ID1 intentionally accepted as user-provided unloaded record; no original edits.

## Separate learning-project handoff (2026-09-26, latest target decision)

- Created sibling /home/daeyun/HRM-Force-Estimation as an independent Git main
  repository; no commits/remotes/push. Documentation and JSON interface examples
  ONLY, no training/no-load-analysis implementation or training runs yet.
- Start at its README.md, AGENTS.md, CODEX_HANDOFF.md, docs/DATASET_SPEC.md and
  docs/TRAINING_PLAN.md. Features/targets/models are configurable proposals.
- User changed default target to **Kalman F/T**, with raw selectable; hrm_base
  aligned XYZ is the default frame. This supersedes earlier raw-default notes.
- Feature groups support angle/tension exclusions and angle-only ablations.
  Small MLP128/64, GRU64, LSTM64 and causal TCN32 are initial model candidates.
- No-load calibration is required future work: verified unloaded intervals,
  sensor/filter-specific baselines and empirical ranges, not mean confidence
  intervals or hardcoded +/-50 mN. Preserve fixed experiment ID and measured force;
  derive load labels separately, mask location loss on no-load/unknown samples.
- Timestamps are excluded from model X but retained for gap/availability checks;
  split physical trials before scaler/windowing. Recorded summary is multi-purpose.
- Verified JSON syntax, 10 feature groups and 4 force variants against actual
  268-column header, ablation dims26/8/22/18/4/48, doc links and model contracts.
  New data directories may move (now record/20260926_f_ext_datasets/); choose exact
  input paths explicitly and do not double-count old/recovered exports.
- Existing HRM source/recording behavior/data/hardware unchanged by this task.

## Renamed-session CSV recovery (2026-09-26)

- User renamed a folder while post-stop export was running. Recovered session
  record/20260926_202928_638928 with existing export_csv into csv_recovered/;
  use its summary.csv (9,949 rows, 268 columns, frozen contact ID18).
- DB3 quick_check ok, 764,516 messages match metadata. Original files untouched,
  original interrupted csv/ and exporting/running metadata intentionally retained.
  31 startup rows omitted per frozen config; later core missing rows=0. Same-row
  odd-tilt/even-pan angles and force alignment [-y,-x,-z] verified.
- Original capture has no tag poses and no crop depth, plus 3 RGB image drops;
  CSV recovery does not fabricate these. Details/checksums in session RECOVERY.md.
- No source behavior changes, hardware commands, ROS replay or recording restart.

## Summary starts at first jointly available training row (2026-09-26)

- User confirmed startup trimming after diagnosis of leading motor blanks.
  Default summary_csv.startup_required_groups=[motor,loadcell,fts,wire,angles].
  Gate requires same-row time-matched finite motorpositions4, loadcellstress4,
  rawforceXYZ, wirelength4 and active relativeangles18. Frozen enabled force
  alignment also requires finite alignedXYZ plus validity. Zero/negative valid;
  tag/tip/FK/acceleration/torque/Kalman/velocity not required by default.
- Only leading rows are omitted. After first complete row ALL existing valid
  angle-anchor rows remain, including later missing signals; no hidden time
  compression or hold/interpolation. Preserve original source/receipt stamps,
  elapsed_s origin and all268columns. Schema/manifest.startup records skipped
  prefix, group reasons, first written stamps and later incomplete-row count.
- No common row => header-only summary statusno_common_start, exportpartial;
  postprocess and recorder reflect incomplete training summary, preservebag and
  per-topicCSV. Legacy profiles missingnewkey useoriginalbehavior; manualexport
  can override --startup-required-groups. Config readstartup; restartrecorderfor
  newdefaults. Existing productionCSV/schema/sessionfiles neveroverwritten.
- Real data /tmp-copy verification: seg15/18/17/16 skip53/59/30/3 rows;
  retained5596/5740/5175/5005. Entire output exactlyequals legacy-export suffix,
  alltagsblank stillstarts; original elapsedorigins and integerstamps retained.
  Source142CSV/schema/metafiles hashesunchanged. No liveROS/hardwarecommands.
  Artifacts /tmp/hrm_summary_startup_audit_i3be2e9_.
- Validation: 429 recorder regression tests passed, then 53 focused startup/
  postprocess tests passed including an additional no-common-start report test.
  record_pkg symlink build, installedconfig readback and diffcheck passed.

## AprilTag recording is optional (2026-09-26)

- User explicitly requested: save tags when detected, record without them too.
  Removed tag-camera CameraInfo and tag0/tag1/tag1_in_tag0 poses from default
  required_topics only. apriltag group, detections, all tag poses, CameraInfo,
  metadata snapshots and CSV columns remain selected/unchanged.
- Tag absence/interruption no longer blocks start or triggers required-data
  incomplete status. Missing tag associations remain blank/unmatched, never
  filled with previous values or zero. Other HRM/sensor required checks unchanged;
  allow_incomplete remains false. No image/angle/control or motor behavior changed.
- Config is loaded at recorder startup. After any current Record Stop and CSV
  export finish, restart Data recorder. Installed recording.json is a source
  symlink, so this config-only change does not need a build. Existing session
  snapshots and custom config files keep their original requirements.
- Verification: 381 recorder functional tests passed, including tag-absent
  startup, optional missing/intermittent tag audits and unchanged mandatory
  sensor rejection. No physical hardware or production nodes were started.

## Crop depth added; separate Record/Stop sessions retained (2026-09-26)

- Latest user choice OVERRIDES the initially approved Pause/Resume idea:
  keep existing Stop then new Record for each uninterrupted trial. No new GUI
  Pause service/button, interval_id column, or temporal-window/training code.
  Explanatory rule: LSTM windows must be made within each file/session before
  pooling; concatenating raw rows first would still create false transitions.
  Non-temporal models do not auto-skip metadata: training code selects features.
- Default image storage is now cropped HRM RGB + cropped aligned raw depth only.
  Existing images/hrm_crop/*.png unchanged; new images/hrm_crop_depth holds
  uint16 PNG (16UC1/mono16) or float32 NPY (32FC1), including original invalid
  values such as zero/NaN. No colorization, depth filtering or quantization.
  No full HRM RGB/depth, tag-camera image or visualization image/cloud storage.
  Numeric rosbag, all numeric CSV/summary (268 columns) and metadata retained.
- Estimator publishes /estimated_segment_crop_depth/{image_raw,camera_info,metadata}
  only when a depth image subscriber exists. DDS callback queues a ROI view;
  existing worker performs conversion/publication, independently of fit success.
  CameraInfo has crop-adjusted K/P principal point, no duplicate ROI offset;
  JSON and per-image index explicitly retain source ROI, full dimensions, scale,
  K/P/R/D, encoding, frame and independent original depth source timestamp.
  Requires matching aligned-depth/color-intrinsic dimensions/frame, not raw
  unaligned depth sampled at color pixels. Estimator math/angles unchanged.
- DepthArchive is bounded/asynchronous, exact timestamp/frame matches calibration
  with a bounded0.25s wait. Metadata-only cache handles pre-Record delivery order;
  no pre-Record images cached. Stop drains accepted RGB and depth before CSV.
  Audit reports drops/errors, calibration errors, internal and leading/trailing
  coverage gaps. Missing optional depth marks incomplete but preserves numeric CSV.
  Default depth_archive.enabled=true, required=false preserves old-estimator
  startup compatibility; operator must restart updated estimator and recorder
  and confirm saved depth counts. Existing crop required check remains unchanged.
- RGB/depth archives retain independent timestamps: offline pairing within each
  session is required; exact exposure synchrony or identical online frame pairs
  are not claimed. Unrecorded prior filter state cannot be reconstructed exactly.
  Initial filter/base calibration/warm-up remain a separate reprocessing concern.
- Validation: estimation_pkg/record_pkg/gui_py_pkg symlink builds passed; all780
  functional tests passed including final coverage/lifecycle additions. Isolated ROS
  domain184 synthetic Record/Stop twice saved72/81 raw depth frames, exact RGB
  and uint16 values (including0/65535), zero drops/errors/calibration errors;
  summary268 columns and old-session file hashes unchanged. No interval IDs or
  pause service. No physical camera/motor/production nodes launched or restarted.
  Artifacts: /tmp/hrm_crop_depth_record_ro3g_wet/sessions/.
- Implementation attempt found installed Humble rosbag Recorder exposes pause
  only in its C++ API, not ROS services; Python binding has record/cancel only.
  Pause-related code/tests were removed after user cancellation. No new C++
  recorder package was created. Do not claim /rosbag2_recorder/pause exists.

## Earlier crop-only PNG profile (superseded by crop depth addition above)

- User selected HRM crop colour only to reduce multi-GB sessions. New default:
  /estimated_segment_crop_image -> images/hrm_crop/*.png (lossless), index.csv
  and manifest.json. Index preserves original source stamp, recorder receive
  stamp, frame, filename, dimensions/encoding and frozen contact_segment_id.
- Numeric bag retained for existing CSV generation/recovery; no image payloads
  are duplicated into it. Full HRM RGB, aligned depth, tag-camera RGB and all
  visualization group topics are excluded by default. All numeric sensors,
  motor/control, angles, tip/FK, tag poses, CameraInfo and TF remain selected.
  Existing csv/summary.csv schema, metadata and previous sessions untouched.
- New image_archive.py uses a bounded16-frame reference queue and a worker for
  PNG conversion/compression/writes; callback does not wait for disk I/O. RGB/BGR
  and mono8 with row padding handled correctly. Sequence+source-stamp filenames
  prevent overwrite with repeated stamps. Drop/write/flush errors explicit.
- RecordNode requires progressing crop data before Record by default, freezes
  operator settings through image drain/export, and waits for accepted PNGs
  before finalization. image_archive_audit.py checks index/count/PNG headers and
  dimensions (not all PNG CRCs/pixels), labels and source gaps. Independent
  image_integrity appears in CSV manifest/postprocess and session data_quality.
  Legacy config without image_archive remains bag-only. Synthetic legacy test
  profiles explicitly opt out; old full-image bag readers/exporters remain usable.
- Crop-only permits ROI-aware 2D mask/skeleton reprocessing (do NOT crop twice),
  not depth-based 3D angle reconstruction or tag redetection. Training consumes
  the retained online angles/numeric signals. Images omitted from old captures
  are not fabricated. Restart Data recorder only after Record Stop/export done
  to load the new defaults; no running production nodes were restarted here.
- Validation: 675 recorder+GUI functional tests passed, record_pkg/gui_py_pkg
  symlink build passed. Isolated domain183 synthetic ROS Record/Stop integration:
  80/80 PNGs, zero drops/errors, source stamps/lossless RGB/contact9 preserved;
  75 summary rows retain odd-tilt/even-pan signs/zeros; numeric-only bag verified;
  both image_integrity/data_quality complete. Sources continuing after Stop
  create no further PNGs. No cameras/motors or production recording launched.
  Artifact: /tmp/hrm_crop_record_roundtrip_esir22zs/sessions/20260926_150934_484627.
  Separate synthetic379x337 noisy RGB benchmark at30Hz:60/60 saved in2.029s;
  full concurrent hardware/estimator load was not benchmarked.

## AprilTag measured size correction (2026-09-26)

- User confirmed 20 mm outer black-border edge. launcher/config/apriltag.yaml
  default size and per-ID sizes for ID0/ID1 changed from0.017 to0.020 m.
  Detector tuning/routing/frames unchanged. Current GUI/recorder descriptions
  and tests updated; historical17 mm captures/results left as historical records.
- Installed launcher YAML resolves to source (symlink); no rebuild needed.
  Restart AprilTag after stopping any active recording to load the new scale.
  No running nodes restarted or live parameters changed in this edit. Existing
  recorded data is not automatically rescaled by this configuration change.
- Verification: 20 launcher routing/config tests passed; installed YAML readback
  confirms default/ID0/ID1=0.020 m and both description JSON files parse.

## GUI IK sine controls (2026-09-26)

- Compact IK sine row below Move Tip: axis amplitudes and phases, common period,
  explicit Start/Stop/status. Pan defaults A45/phase90, Tilt A45/phase0, period20s.
  Centres fixed0; A0 means target0, not hold current bent angle. Existing backend
  model/limits unchanged. Parameters sent atomically, then Start only on success.
- No automatic mode/output enable/speed-limit change. GUI Kinematics selection
  gates Start; Stop always accessible. Uses /robot_control/set_parameters_atomically,
  /kinematics/sine_motion and reliable transient-local /kinematics/sine_status.
- Async Qt polling with 3s deadlines; pending config/Start futures retained to
  prevent late-write/late-Start races. Stop cancels continuation; an already-sent
  Start is followed by another Stop after late completion. Unknown result does
  not claim stopped or allow another Start. No automatic repeat Start.
- Stop retains last motor target, no zero return, NOT hardware emergency stop.
  GUI closing is not a guaranteed Stop for an externally managed controller.
  README and docs/IK_SINE_MOTION.md updated; actual motors must not run in tests.
- Validation: all 365 GUI functional tests passed (31 new sine cases), symlink
  build and scoped lint passed. 1366x768/1500x900/1920x1080 layouts fit without
  scrolling; tightened right-panel vertical spacing4->2, vertical margins6->4.
  Isolated localhost ROS domain182 exercised actual C++ controller with the Qt
  panel: pan-only, tilt-only and dual-axis atomic config/Start/Stop all passed.
  Output gate stayed false; motor_command remapped to test-only topic; zero motor
  commands observed. No TCP, sensors, cameras or actual motor motion. Test node
  shut down after completion; production graph/configuration untouched.

## Skeleton preview estimated and FK tips (2026-09-26)

- HRM skeleton canvas top now shows hrm_base | XYZ [mm], Est and FK positions
  (signed one decimal). Uses /estimated_tip_position (hardware-length-constrained
  final curve boundary) and /kinematics/fk_tip_position (measured-angle FK).
  Original messages remain metres. No unscaled/segment-centre substitution.
- New skeleton_preview.py caches eight image and eight PointStamped references
  per stream. Prefer newest complete exact-source-stamp image/Est/FK set with
  every receipt <=0.2 s old; otherwise latest image + independent matched rows.
  Camera frame on image intentionally differs from required hrm_base on points.
  Missing/invalid/stale/frame-mismatched values clear; conversion errors suppress
  positions. No indefinite cached values or nearest-time mixing.
- Small read-only subscriptions use existing preview executor/QoS. Point-only
  arrival updates text without reconverting the same RGB image. ROI receiver
  does not subscribe to tips. Uses existing in-canvas header, no extra scrollbar.
  Estimation, robot controller, recording and physical motor state unchanged.
- Verification: 331 GUI functional tests passed, including 49 new skeleton-tip
  tests; gui_py_pkg symlink build passed. Offscreen 1366x768 GUI screenshot
  checked: both rows fit inside the existing 246x136 skeleton canvas. No real
  camera, controller or motor launch/commands were used for these tests.

## GUI live exposure and compact control spacing (2026-09-25)

- New camera_exposure.py adds one compact row inside System/Packages: HRM/Tag
  target, Auto checkbox, manual ms value, Read and Apply plus tooltip status.
  Uses existing camera parameter services, not a device handle or launch restart.
  Reads actual available module/range/step; RGB driver unit 0.1 ms, D405 depth/color
  unit 0.001 ms. AE-ON value is stored manual setting, NOT actual automatic exposure.
- Explicit Apply only: re-read module/range, AE OFF ACK then exposure ACK for
  manual; Auto ON writes only the bool. Get readback must match before Applied.
  Services/polling nonblocking, per-phase 3 s timeout, stop follow-up writes after
  failure/timeout/target change. Partial failure asks user to Read; no rollback.
  Startup/Read/editing never writes. No gain/FPS/priority changes. Settings live
  only; launch YAML defaults remain unchanged on camera restart. D405 affects
  depth as well as color. No motor changes or automatic enabling.
- Force axis groups now natural-width label/selector with 4 px internal and 16 px
  inter-group gaps; operation/actuation choices grouped left with 12 px gaps.
- Corrected stale camera-selector test/README expectation D435i -> existing GUI
  D455 default; actual camera configuration and launcher defaults unchanged.
- Live read-only verification found both camera nodes had been stopped by then;
  GUI correctly timed out without blocking. No hardware settings were changed.
- Verification: 282 GUI functional tests passed; gui_py_pkg symlink build passed.
  Simulated ROS parameter services in localhost-only domain 173 verified actual
  Read, manual 8 ms (AE false then RGB raw 80), and AE true-only apply/readback.
  No real devices, full GUI autostart, recording or motor commands in that test.
  Layout tests cover 1366x768/1500x900/1920x1080, including exposure row visibility.

## Tag preview relative-pose display (2026-09-25)

- Rightmost Tag camera / detections canvas displays ID0 -> ID1 XYZ [mm] and
  RPY [deg], signed one decimal, using /apriltag/tag1_in_tag0/pose (already
  launched by AprilTag component, independent of recording).
- RPY uses extrinsic XYZ, Rz(yaw) Ry(pitch) Rx(roll), not HRM joint angles.
  Tags remain uncompensated for mounting offsets. Tooltip explains convention,
  Euler wrapping/singularities, and exact-image matching.
- Bounded eight-message pose cache; require ID0 parent and exact displayed
  image stamp with both matched detections. Missing/invalid/stale (>0.2s)
  values clear to a status, not zero or held values. RGB fallback has no pose.
- Draw text inside existing top dark margin; scale full image below if needed.
  No extra layout row/scroll, detector, package or topic publisher. Pose-only
  arrivals update text without re-converting image pixels. Hardware untouched.
- Verification: 92 targeted overlay/image/layout tests passed; gui_py_pkg build
  passed. Offscreen full GUI remains 1366x768, tag canvas 246x136, no scrollbar;
  synthetic screenshot inspected. Camera/motor runtime not launched/tested.
  Wider GUI run: 253 passed, 1 pre-existing camera-selector failure (expects D435i
  default while current user config is D455); 2 more regression tests were added
  afterwards and covered by the 92 targeted passes. Camera defaults/test unchanged.

## User-selected summary columns (2026-09-25)

- Current summary.csv has 268 columns (supersedes the 331-column version below).
  Remove estimated_tip_unscaled position + association metadata (9 columns) from
  summary only; bag/topic CSV unchanged. Keep all other requested groups, including
  raw/Kalman/aligned F/T, original pan/tilt relative/absolute, FK point and full pose,
  estimated tip/orientation, contact label and selected relative_angle_1..18.
- Replace summary's 72 velocity fields with relative_angular_velocity_1_rad_s
  through relative_angular_velocity_18_rad_s: odd tilt / even pan, same source
  sample, sign/zero/unit retained. All four original velocity arrays remain in
  angle topic CSV/bag. No estimation, filtering, control or training changes.
- ID0-relative ID1 XYZ and quaternion/Euler already have separate scalar columns;
  tag mounting offsets remain uncompensated. Times and association flags retained.
  Missing/mistimed groups blank only their cells; bad/nonincreasing angle anchor
  stamps skip summary rows and are counted. Source records stay intact. A network
  does not automatically ignore missing inputs or bad ground-truth labels.
- Cable summary is actual motor-feedback-converted displacement/velocity, NOT IK
  target. /kinematics/target_wire_length is separately recorded, not in summary.
- User reports about 10g startup pretension variation vs roughly 1000g loaded
  cable tension. Recommended keeping actual four-channel tensions and recording a
  consistent unloaded baseline, testing held-out sessions with baseline variation.
  No automatic tension zeroing or baseline subtraction/storage added this turn.
  Physical pretension variation differs from sensor-zero error; neither maps
  directly to equal relative error in external-force estimates.
- Verification: 206 recorder functional tests passed (including 40 targeted
  summary/label tests); record_pkg symlink build passed. Independent read-only
  review found no blocking issue. Tests used synthetic/offline bags; no actual
  sensors, cameras or motors were launched, and existing captures were not edited.

## Configurable IK sine motion (2026-09-25)

- New SetBool service /kinematics/sine_motion (NOT legacy /motion/move_sine_wave).
  Parameters motion.sine.tilt/pan = [center_deg, amplitude_deg, phase_deg],
  motion.sine.period_sec=20, motion.sine.max_speed_deg_s=15. User-requested defaults:
  tilt [0,45,0], pan [0,45,90] (both +/-45deg, pan leads 90deg). SineConfig is the
  single source of defaults; explicit ROS overrides still take precedence.
  User clarified 5deg/s was not their requirement; it was assistant-added. Period20
  needs peak14.14deg/s at amplitude45, so default/allowed max speed is now15deg/s.
  Same-period single-axis, dual-axis and 90deg quadrature supported. These are
  aggregate SurgicalTool IK commands, not per-joint angles or Cartesian circle IK.
- Explicit start only; no auto motor enabling/mode switch. Uses existing IK cable
  conversion and motor gate. Max envelope +/-45deg/axis, speed <=15deg/s configurable,
  validates analytical peak speed <= configured speed. Approaches initial phase
  from LAST IK COMMAND with rate limiting (not a measured pose). Real-output Start
  rejects actual motor positions >1000 counts from that calibrated IK starting pose.
- Stop cancels new targets, no zero return/release/compensating motion. Drive may
  continue to last target: this is NOT a hardware emergency stop. Explicit restart.
- Real-output path requires 4 motor/loadcell channels fresh within .25s (receipt
  and source stamps) below TENSION_LIMIT; timer stalls/motor software range violations
  stop too. Dry-run does not require devices. Mode/output gate changes cancel motion.
  Manual IK/direct motor and legacy motion starts rejected while new sine is active.
- /kinematics/sine_status added as optional recorded topic. Existing pose/wire command
  topics + runtime parameter snapshot/events record the waveform. GUI buttons were
  added Sep26; commands/limitations in docs/IK_SINE_MOTION.md. IK logs throttled to 2s.
- Builds: robot_control_pkg, record_pkg passed (pre-existing compiler warnings).
  C++ FK/position/sine test targets passed; all 200 recorder functional tests passed.
  Isolated domain79/localhost-only smoke with remapped motor output tests tilt/pan,
  conflicting commands, validation, Start/Stop, gate/mode changes, start pose mismatch,
  over-tension and stale feedback; actual devices were NOT launched/moved.
  Test script: src/robot_control_pkg/test/ik_sine_smoke.py.

## Operator contact label and additional relative columns (2026-09-25)

- GUI contact_segment_id QSpinBox (0..18) shares the recording options row.
  Settings are atomically acknowledged before Record; recorder validates type/range
  and freezes ID during capture/export like force alignment. Change between sessions.
- session.json snapshot stores the ID; all per-topic time-series CSV rows and
  summary rows contain it. Legacy missing labels stay blank, never default to 0.
  session_metadata.csv keeps its existing key/value shape with the snapshot entry.
- experiment_labels.py selects relative_angle_1..18 by the explicit user-requested
  confirmed odd tilt / even pan rule, same angle sample, signed radians including zeros.
  Initially summary appended 19 columns (312 -> 331); current selection above is 268.
  Angle topic CSV adds these label/relative-angle fields too.
  All original pan/tilt arrays, timestamps, forces, bag messages are preserved.
- User confirmed tilt-first after the original request reversed it. Export now
  matches estimator's odd tilt/even pan. Estimation, FK and hardware are unchanged.
  Previously exported CSVs are not rewritten; re-export old bags to a NEW directory
  when corrected relative_angle columns are needed.
- 130 targeted GUI/recorder/CSV tests passed (synthetic/offline data only), including
  0/negative angles, sign/zero preservation, changing force with fixed ID, 331-column
  header/data consistency, legacy blank labels, ACK and lock behavior. No motors run.
- record_pkg + gui_py_pkg symlink builds passed. Wider functional suite: 408 passed,
  1 unrelated camera-selector test failed because it expects D435i but the existing
  configured default is D455; camera configuration/test was left unchanged.

## GUI sensor presentation and serial auto-selection (2026-09-23)

- F/T numeric torque display and both topic/summary CSV sensor tx/ty/tz use
  one decimal (raw + Kalman topics). Internal computation, rosbag, forces and
  motor torque retain precision. Existing CSV files are not rewritten.
- F/T canvas now has force above torque, separate adaptive y scales, fixed
  upper-right legends; same layout footprint.
- Serial ports refresh at GUI construction and Serial Start. Unique USB/ACM
  device auto-selected when empty/previous device absent; multiple require a
  choice. No automatic reconnection of running/external readers.
- 79 targeted GUI/CSV tests passed without hardware or process launches.

## Wide inspection CSV; learning remains a separate repository (2026-09-23)

- User explicitly keeps training/data-array construction in another Git repo.
  HRM repo performs sensing, estimation, recording and human inspection only.
- Added record_pkg/summary_csv.py and export_summary entrypoint. Record Stop's
  export_csv now adds csv/summary.csv + summary.schema.json, retaining every
  original topic CSV. Default configurable ±25 ms nearest association anchored
  on angle source stamp; sensor/tag values outside window blank, not zero/hold.
  Stamped tips/DH orientations require EXACT angle source stamps. Headerless
  wire data use ANGLE RECEIVE time and are explicitly marked receive-based.
- Motor/loadcell/cable channels 1..4 are scalar columns. All 4 pan/tilt
  relative/absolute arrays (18 columns each, rad) and all 4 angular velocities
  are expanded without deleting inactive relative-axis zeros. Physical order
  remains tilt-pan; absolute arrays are Base direction angles, not relative sums.
- Tag columns are raw ID0->ID1 XYZ(m), xyzw, extrinsic XYZ RPY(deg). No marker
  mounting compensation or frame relabel. If detections CSV is available, an
  empty/one-tag nearest frame blocks pose reuse even if a neighboring valid pose
  is within 25 ms. Pose must match chosen detection stamp exactly.
- estimated_tip position is final fitted boundary; unscaled preserved separately.
  Last-center TF supplies ONLY DH orientation, never its center as tip position.
  FK PointStamped and hrm_fk_tip TF position/orientation both preserved in distinct
  fields. These rotations share filtered joint angles, not independent sensing.
- Confirmed offline on recorded .../20260921_123441_835414/csv: new summary.csv
  565 rows, 312 columns; all sensor/tip groups matched565, tag563 +2 blank rows.
  No existing CSV/bag overwritten, no live hardware/nodes/commands used.
- record_pkg build and export_summary entrypoint verified. 105 focused recorder/
  export/summary regressions passed, including real offline bag export with
  summary enabled/disabled, explicit tag-miss blocking and original CSV preservation.
- GUI F/T plot legend now fx,fy,fz,tx,ty,tz with upper-right location retained;
  corresponding 8 offscreen layout/legend tests passed. Operator restarts GUI.

## Passive live tip comparison and recorder bottleneck (2026-09-21)

- User connected D455 tag camera, all motors/sensors, requested separate tag/
  estimated/FK tip CSVs and angle comparison, explicitly NO motor movement.
  Only passive subscriptions, parameter reads and owned rosbag processes used;
  existing nodes/parameters untouched. Zero motor-command messages observed.
- Added scripts/capture_tip_validation.py for bounded passive capture using
  record_pkg CaptureSession/exporter. --telemetry-only excludes images/cloud
  while retaining markers, TF, source stamps, angles, sensors and motor states.
  Production required_topics and GUI recorder unchanged.
- Primary session record/tip_validation/20260921_123441_835414: 20 s requested,
  563 tag relative poses, 565 estimated/FK tips (~30 Hz), 3963 loadcell and 3964
  raw/Kalman F/T samples (~200 Hz); 4796 motor samples. Bag+CSV 43 MB, continuity
  complete, no cache-loss warning. Live FK PointStamped and hrm_fk_tip TF both
  available; initial ros2 daemon topic list was incomplete, fresh discovery fixed it.
- IMPORTANT full-image record failure: first session .../20260921_123302_373699
  retained incomplete (~1.5 GB). rosbag cache reported 27,873 dropped messages;
  independent observer still got all 600 tag/estimated/FK samples in 20 s.
  Bottleneck is current full-image storage path (64 MB cache/resilient sqlite),
  not evidence of camera-rate loss. Must address before full training capture.
  Do not label successful CSV export as complete data: integrity is separate.
- Calibration remains user's task. Tag pose frame ID0 != hrm_base; require
  base_T_ID0 * ID0_T_ID1 * ID1_T_tip including rotations/lever arms. Raw absolute
  tag-minus-estimate error is invalid before compensation. Last-center TF
  orientation and FK tip orientation share filtered DH angles, not independent.
  See docs/TIP_VALIDATION.md for capture provenance and interpretation.
- scripts/analyze_tip_capture.py provides passive offline CSVs in analysis/.
  565 exact-source depth/FK pairs: distance mean0.2473/RMS0.2945/max0.9067 mm
  (not GT error). Both tags detected in565/565 arrays. First-pose-relative angular
  change max tag1.282° vs FK7.090°; not calibrated orientation error. Tangent vs
  FK +X direction difference mean2.221°/RMS2.793°/max9.140°. Source poses,
  quaternions, extrinsic XYZ Euler angles, last-boundary tangent exported with
  original source/receive stamps and no missing-value fill. See session RESULTS.md.
  Offline analyzer 18 tests and focused lint passed. Owned capture/recorder nodes
  closed after testing; operator cameras/controller/sensors were left running.

## Source-stamped depth-fit and FK tip recording (2026-09-21)

- Added PointStamped outputs in metres, hrm_base, preserving the input image
  stamp: /estimated_tip_position is the final fitted boundary P19 after the
  existing 82.27 mm hardware-length scaling; /estimated_tip_position_unscaled
  preserves the fitted endpoint BEFORE that scaling. Unscaled is NOT raw depth
  or independent ground truth; base anchoring and curve fitting still apply.
  No extra depth lookup, curve/angle/filter changes, or extra image processing.
- robot_control adds /kinematics/fk_tip_position using the same FK result as
  /tool_endeffector_pose, but copies the accepted SegmentAngle header. Only new
  angle samples publish, not control-loop repetitions. Legacy output retained;
  no control/motor behavior changed. FK is model-derived, not separately sensed.
- Three topics added to the default recorder groups as OPTIONAL selections;
  required_topics unchanged. Default selected count is now 43 (34 full-field
  CSV, 8 image/cloud metadata-only CSV, 1 MarkerArray bag-only). Record Stop
  exports separate CSVs with point.x/y/z, frame, source_time_ns and receive time.
  No interpolation/hold-last added; missing samples stay missing. Restart
  estimator, robot_control and recorder to use the new outputs/config.
- Tag reuse was discussed, NOT implemented. Moving ID1's last pose must not
  masquerade as a fresh ground-truth measurement. Future display-only hold
  should expose held/observed, age and original stamp. Fixed camera + fixed ID0
  could instead use an explicitly calibrated reference, reset after remounting.
- Depth-fit endpoint can serve directly for position observation; angles retain
  full-body shape information for force learning, and FK enforces link/axis
  assumptions. Both estimates share depth input, so agreement is NOT independent
  accuracy validation. Tag comparisons need source-time/frame alignment plus
  ID0-to-hrm_base and ID1-to-physical-tip mounting transforms. Surface depth is
  not automatically a physical centerline measurement.
- robot_control, estimation and record builds succeeded. Synthetic metric
  reconstruction/publisher tests and actual offline rosbag-to-CSV roundtrip
  verified distinct tip series, units, source stamps and no missing-row fill.
  No live camera, controller or motor nodes were started for this work.

## Fixed plot legend and recording audit (2026-09-21)

- GUI F/T graph legend now uses loc='upper right', not Matplotlib's automatic
  'best' placement. No sensor/filter/control behavior changed. Regression draws
  changing traces/limits and verifies the legend's axes-relative bounds remain
  fixed. GUI build succeeded; 72 layout/recording regression tests passed.
- User asked how to record selectively and what CSV/missing data mean. Recorder
  settings were inspected, NOT changed. enabled_groups + extra_topics select
  storage; required_topics alone gate startup/quality, and must be a subset of
  selected topics. JSON changes require recorder restart; no GUI selector yet.
  allow_incomplete=true only bypasses startup requirements, not quality reporting.
- At that audit, config selected 40 topics: 31 full-field CSV, 8 image/cloud metadata-only
  CSV, 1 MarkerArray bag-only. Only actual received messages create rows/files;
  arrays are JSON cells, each topic has independent timestamps. No recorder
  forward-fill, zero-fill, resampling or automatic sensor synchronization.
- Missing AprilTag: individual detected tag pose only; relative pose requires
  matching parent AND source timestamp for both IDs. /detections can record []
  while pose CSVs have no row at that instant. Runtime required gaps >2 s warn
  and continue; 10 s startup readiness timeout stops only the recorder. Short
  missed frames do not necessarily fail the configured continuity audit.
- Important existing upstream issue FOUND, NOT FIXED: tcp_pkg/src/tcp_node.cpp
  recvmsg() logs recv==-1 but still decodes/publishes; partial packet lengths
  aren't validated. Old/mixed buffer data can get a fresh header stamp and be
  recorded as new /motor_state. Needs separate TCP receive/framing repair
  before trusting communication-drop datasets; recorder cannot infer validity.
  Also /loadcell_state_offset still contains legacy two-channel offsets, unlike
  four-channel /loadcell_state. No changes to either producer in this task.

## GUI tag overlays and compact right panel (2026-09-18 late night)

- User requested a rightmost live tag-camera tile, preferably detected outlines
  without another package, and no right-side motor/record scrolling. Completed
  in image_preview.py/gui_node.py; existing motor safety, command services,
  filters, zeroing and recording behavior unchanged. No hardware motor commands.
- Three bottom-right tiles: aligned depth, skeleton, tag RGB. Tag overlay reuses
  apriltag_msgs/AprilTagDetectionArray /detections and color/image_rect. Exact
  (frame_id, sec, nanosec) pairing; 8-message caches, <=0.2 s receipt age for both
  image and result. Draw after downscaling to tile width <=640 px, Qt ~30 Hz cap.
  Image callbacks remain
  on separate executor; no redetection or extra image publisher. Existing ROI
  LatestImages(topics=...) API preserved. Declared already-installed apriltag_msgs.
- If rectifier absent for 1 s, dynamically subscribe to tag raw RGB and show
  Raw RGB—no overlay; remove fallback when rectified stream returns. Empty tag
  arrays clear outlines; waiting/mismatched/stale input statuses explicit.
- Right QScrollArea removed (not merely hidden): compact motor/filter rows,
  Record+status on one row, recording below motor commands, 10 pt desktop font.
  Left System above Sensors and raw-sensor scroll preserved. All controls and
  three tiles in bounds at 1366x768, 1500x900, 1920x1080, including two-line status
  and force alignment ON/OFF. Extremely small screens/custom font scaling not
  guaranteed. Original enable gate stays visible at top and OFF; sensor zero
  stays under left sensor readings.
- gui_py_pkg build succeeded; 316 focused tests passed (layout, overlay, ROI,
  GUI, launch, recorder, probe). Targeted module/test lint and diff checks passed.
- Live camera audit: user GUI210949, D405210991, D435i211023 already running;
  cameras left untouched. Depth/RGB ~30 Hz. Estimator/AprilTag were off.
  Started only owned AprilTag temporarily, and separate offscreen read-only preview:
  depth/tag render ~28.6/28.5 Hz, refresh mean 3.78 ms, p95 5.52 ms. Current RGB
  scene very dark; detections empty. No claim of live ID0/1 detection in this test.
  Raw fallback also worked with detector off. Existing GUI DDS env unchanged;
  new preview tested without injected DDS profile to match production conditions.
- Replayed earlier real illuminated D435i frame + CameraInfo from
  /tmp/hrm-d435i-check.VwFmmx/run/ in isolated empty domain96 through real apriltag:
  GUI showed ID0 green / ID1 yellow outlines on 296 matched frames, zero header
  mismatches, mean 1.37 ms tag-only refresh. Static replay is NOT fresh live or
  accuracy validation. Artifacts: /tmp/hrm-preview-replay-i88wkgkv/preview/,
  /tmp/hrm-tag-gui-overlay.NEqDbb/run/ (dark live),
  /tmp/hrm-tag-gui-raw.F1YbCG/run/ (raw fallback). Test-owned processes closed.
- scripts/test_gui_tag_preview_live.py is a read-only bounded actual Qt probe;
  --require-overlay requires ID0+ID1. No cameras/motors/recorder started by it.
  GUI README updated. User must restart GUI when ready; current user GUI is not
  hot-reloaded, and closing it stops its owned cameras/components, so it was
  not closed automatically.

## Dedicated D435i AprilTag camera (2026-09-18 night)

- Latest request: use D435i for AprilTag, retain D405 for HRM, use existing CUDA
  librealsense and give both camera rows independent GUI model/serial selectors.
  User authorized installing dependencies and live testing while away. No motor,
  controller, estimator or recorder nodes are started for this verification.
- User physically recovered D435i USB3; SDK query and camera launch now confirm
  D435I serial949122070024, FW5.17.3.10, USB3.2 on port2-6. D405 remains USB3.2,
  serial123622270472 on2-8. Older unresolved D435i USB notes below are superseded.
  No firmware/reset changes in this task; D455 is not connected now.
- New launcher/cuda_apriltag_camera.launch.py uses isolated name+namespace
  tag_camera, D435i$ exact-model default. Shared CUDA launch now allows scoped
  camera_name/camera_namespace/stream_config arguments, rejects YAML identity
  or frame overrides, and retains D405 defaults. RGB1280x720x30 only; depth,
  alignment, infrared and IMU off. CUDA SDK use does not GPU-accelerate the CPU
  apriltag_ros detector or image_proc rectifier.
- GUI adds Tag camera (CUDA) on its existing component description line with
  D435i default and independent serial/locking/Start/Stop; HRM defaults D405.
  No new System scroll area. Automatic schedule inserts tag camera at0.5s;
  original component delays and global auto-start=false default unchanged.
- AprilTag launch defaults to /tag_camera/tag_camera/color/image_raw plus
  matching camera_info. image_proc::RectifyNode produces color/image_rect using
  K,D,R,P, then existing apriltag_ros3.2.2 uses P. Original image stamps and
  tag_camera_color_optical_frame are retained; no crop or fake inter-camera TF.
  ID0/ID1,17mm,decimate1,hamming0 and /detections,/apriltag/... outputs preserved.
  Single-D405 fallback must explicitly use rectify:=false and old image/info.
- image_proc dependency is staged in ignored .ros_image_proc/root/opt/ros/humble
  (official image-proc + tracetools-image-pipeline3.0.9). sudo requires an
  interactive password; no system APT/keyring/library edits. Old cached package
  index/signing key were stale; refreshed official same-fingerprint ROS key,
  InRelease signature, index and debSHA256 verified. Reproducible installer:
  scripts/install_image_proc_local.sh; --verify-only checks existing runtime.
  AprilTag launch prefers system dependency, otherwise prepends local prefix
  only to its process and children. Git does not transfer unpacked dependencies.
  Rectifier independently enables the image-sized FastDDS profile (the camera
  process cannot supply this to a separately launched rectifier). Explicit user
  DDS profiles and non-FastDDS RMW implementations are respected.
- Recorder's AprilTag group now includes required second raw RGB+CameraInfo,
  second-camera parameter snapshot and profile YAML. Existing tag/relative CSV
  schema unchanged. ID0-to-base and ID1-to-tip mounting calibration and camera
  source-clock alignment remain required for comparison; not solved by this wiring.
- record_pkg/gui_py_pkg/launcher symlink builds succeeded. Focused GUI/launch/
  recorder/probe batch293 passed; compact fullGUI offscreen render at1500x900.
  New scripts/test_d435i_apriltag_live.py is a bounded exact-serial test in an
  empty isolated ROSdomain; it starts only its own camera+detector and cleans
  only its owned process groups. Optional camera params queried individually:
  undeclared enable_infra must not invalidate the required identity readback.
- LIVE VERIFIED22:41: D435i RGB1280x720, CUDA SDK library loaded from workspace;
  raw/rect/info/detections each450 unique frames in15s,29.978Hz. ID0 and ID1
  detected together450/450; both stamped poses and ID1-in-ID0 each450 at29.978Hz.
  Raw-to-rect and rect-to-detection source stamps matched450/450. Pose sampling
  windows shifted by one boundary callback (449/450 with detector,450/450 between
  relative and tag0), not evidence of a timestamp rewrite. Frame labels correct.
  This camera's reported D coefficients were all zero; rectification correctly
  passes through that calibration. Do not invent nonzero lens distortion.
  Scene was static; this does NOT establish dynamic robustness or1mm accuracy,
  and D405/estimator simultaneous load was not tested. Both owned launches
  exited0; only preexisting RealSense Viewer remained. No physical motion.
  Report+logs+RGB snapshots: /tmp/hrm-d435i-check.VwFmmx/run/ (temporary).

## HRM CUDA camera selection (2026-09-18 evening)

- User paused D455 USB troubleshooting to prevent the HRM launch from grabbing
  the wrong camera when multiple RealSense devices are connected. No device
  reset, firmware update or camera/motor restart was performed in this change.
- GUI camera package row reuses its existing description line for a model
  combo (D405 default; D435i/D435/D455/D415 alternatives) and optional SDK serial.
  No extra settings row or scroll area. No SDK enumeration on the GUI thread.
  Inputs lock while starting/running/stopping/external; changes require STOPPED
  and next Start. Each GUI session defaults to D405 without a serial filter.
- New camera_selector.py and system_components.json feed device_type/serial_no
  to existing launcher/cuda_realsense.launch.py. Terminal script and combined
  cuda_estimation.launch.py also default to D405. Exact model suffix matching
  avoids D435 selecting D435i; optional serial is ANDed with model, with no
  fallback to a different model/device on mismatch. Leading zeros survive via
  the driver's underscore string prefix. Invalid camera serial blocks only
  camera Start, not other components.
- Original CUDA library paths, vendor launch Include, Fast DDS setup, D405
  stream profiles and /camera/camera topics are retained. Vendor YAML has
  higher precedence than launch params: wrapper rejects identity keys in the
  stream config so future YAML edits cannot silently bypass model/serial filters.
  This selects one HRM camera; independently launching/routing a second tag
  camera is still separate work. Changing cameras also requires checking
  supported profiles, intrinsics/ROI and restarting estimator base/filter state.
- gui_py_pkg + launcher symlink builds succeeded (only an unused explicit
  PYTHON_EXECUTABLE CMake argument warning). Installed launch --show-args and
  installed GUI selector/config imports verified. 224 focused GUI/launch tests
  passed, including actual vendor ROS parameter evaluation without executing
  its Node. Offscreen full GUI render at 1500x900 verified compact placement.
  No live camera start/mismatch/reconnect test was performed.
- USB diagnostic context before this task: D455 SDK serial 043422251095,
  firmware 5.15.1, USB2.1; D405 SDK serial 123622270472, firmware5.16.0.1,
  USB3.2. USB sysfs serials are ASIC IDs (031323061979 / 124423070627), NOT the
  SDK serial_no parameter. D455 briefly enumerated SuperSpeed then disconnected
  with USB -71; current USB2 path went through ASMedia ASM107x. User verified
  USB3 cable/port and flipped connector. Do not repeat cable-only blame.
  Initial D455-only Hardware Reset proposal was not executed during GUI work.
- Subsequent user request explicitly authorized recovery of D455 or D435i.
  At 22:03 both already had FW5.17.3.10 (not flashed by this agent). D435i SDK
  serial949122070024, ASIC950323052663, path1-7. Performed one SDK hardware
  restart and one OS USBDEVFS reset for EACH exact D455/D435i, never D405/hubs/
  xHCI/motors. Final22:10: D4551-2.1=480Mbps, D435i1-7=480Mbps, D4052-8=5000Mbps.
  D435i's SuperSpeed peer usb2-port6 and D455's peer2-9-port1 are enabled,
  runtime-active, not attached; no evidence of a disabled/suspended SS port.
  USB3 recovery remains UNSOLVED; next discriminating test requires physically
  moving D435i to the known-good D405 PC port after safely stopping its users.
  No firmware downgrade/upgrade or persistent USB power changes were made.
  Temporary exact-serial-only recovery helper: /tmp/hrm-usb-recovery.M6N1hh.
  D455 RGB fixed focus confirmed in official D400 Oct2025 datasheet Table3-19;
  RGB minimum focus distance not established (do not confuse depth MinZ with
  RGB focus). Asked user actual tag distance; pending. No focus/image changes.

## Isolated CPU / Isaac AprilTag comparison (2026-09-18 evening)

- User asked to compare apriltag_ros versus isaac_ros_apriltag and is preparing
  the physical scene. A readiness question was sent; fresh controlled ID1
  capture remains pending. Do not silently switch the production detector.
- Isaac ROS3.1 CUDA detector now RUNS on RTX3060/CUDA12.4 without system install.
  Official23debs + VPI3.1.5 extracted only into ignored `.isaac_ros_compare/root`,
  verified SHA256 from official indexes. negotiated+interfaces built locally
  commit eac198b55dcd052af5988f0f174902913c5f20e7. COLCON_IGNORE excludes stage.
  See manifests/staging scripts there; files are not transferred by git.
- New scripts/with_isaac_ros_compare.sh sets child-only ROS/library/Python paths;
  compare_apriltag_backends.py + apriltag_backend_probe.launch.py replay saved
  frames in emptydomain95 with all topics/TF private, ownprocess cleanup only.
  No camera/motor/globalparameter or productionconfig changes. CPU onebitdamage
  smoke passed; GPUclean/blank smoke passed. Pure metric/pose/cleanup tests5passed.
- New scripts/capture_apriltag_comparison.py provides repeatable read-only RGB +
  exactstampCameraInfo capture; newoutputdirrequired, boundedqueues, constant
  intrinsics/frame/dimensions checked; incompletecapturesnonzeroexit+metadata.
  No freshlivecaptureperformedwiththishelperyet; userreadinessstillpending.
  Capture27 + backend5 =32 focusedtests passed; scopedflake8max100 and shell
  syntax passed. Read-only hostprocesscheckconfirmed no comparisonchildrenleft.
- Same newer90RGB848x480frames `/tmp/hrm_apriltag_compare_lo3t4aw4/capture.npz`:
  CPUdecimate1/h0,CPU1/h1,IsaacGPU ALL ID0=90/90,ID1=0/90,zeroresponsetimeouts.
  Meanroundtrip15.59/15.56/3.60ms (notpurecomputelatency/liveFPS). Report
  `/tmp/hrm_apriltag_backends_existing90.json`. ID1robustness NOT improvedhere.
- Newersmalltag wasnotoccluded (unlikeearlier180frameview); small/blurredpattern
  remains a possiblecause, notprovenalone. ID0poseconsecutivemeansteps:
  CPU.012mm/.029deg vsGPU.789mm/1.169deg, absoluteorientationsalsodiffer. Do not
  claimGPUimprovesposequality or wireitsTF into recorderwithoutframeaudit.
- RawIsaacID0 results have2corner/poseclusters; onecornerx334/335 toggles,
  twoposesdiffer3.695mm/5.476deg. AllIsaacreturnedcornersintegersinthissequence,
  CPUsubpixel; adapters/harnessdon'tround. No universal/newerversionclaim.
- Details/versionconstraints/repeatcommands docs/APRILTAG_BACKEND_COMPARISON.md.
  Minimal stagedsetup lacksCV-CUDAforunrelatedimage_procnodes; AprilTagpathworks.
  Experimental unpackedprefix, not NVIDIArecommendedDocker installation.

## AprilTag decimate / hamming comparison (2026-09-18)

- User requested decimate=1.0 trial and max_hamming=1 comparison because ID1
  itself disappears. Captured 180 full RGB848x480 frames across17.905s from the
  livecamera, each exact-stamp CameraInfo matched; sampled~10Hz while camera
  callbacks~30Hz. Active originaldetector was2.0/h0; no record/motor nodes seen.
- New scripts/compare_apriltag_settings.py tests identical images in empty
  isolated ROSdomain93 using installedapriltag_ros3.2.2 C++ detector; all topics
  private /apriltag_compare. 2.0/h0,1.0/h0,1.0/h1 ALL returned180responses,
  ID0=180/180,ID1=0/180. No ID1 pose comparison possible. Publisher-to-response
  mean round trips5.10/14.20/14.07ms are NOT pure detector latency/liveFPS.
- Changed launcher/config/apriltag.yaml decimate2→1 only; max_hamming stays0
  because no advantage measured. Active /apriltag_node atomically accepted
  decimate1 and get-parameter readbackverified1/h0. Sixsecondlivefollow-up:
  180unique detectionarrays (~30Hz),ID0all,ID1none. No camera/node restart or
  motor commands. Installedconfigsymlinkverified; no build needed forYAML.
- ID1 failure NOT resolved. Firstimage shows another small angledtag partially
  behind HRM (suspectedID1; askeduser toconfirm); largebottomtagID0isvisible.
  Need fulltagborders visible before glare-only A/B. Do not claim reflections,
  hamming filter, or insufficient compute is a proven solecause.
- Syntheticclean/one-bit-damaged-ID1/blank testverifiedcounts1/3,1/3,2/3
  for2/h0,1/h0,1/h1 respectively; admittedextraresult hamming1. Test responses
  vs missing tags handledseparately; ownchildrenonly cleanup,3timeoutsabort.
  Details and repeatcommand docs/APRILTAG_TUNING.md. Temporaryartifacts in
  /tmp/hrm_apriltag_compare_11b7bizt (capture.npz,metadata,first_rgb,comparison).

## GUI estimator ROI selection (2026-09-18)

- AprilTag input remains the full /camera/camera/color/image_rect_raw plus full
  color/camera_info. Estimator receives that full RGB stream but internally
  crops RGB and aligned depth with the same ROI; original pixel offset is
  retained for depth deprojection. No AprilTag, calibration, FK or filter math
  changes in this task.
- System → Estimation ROI… opens a one-shot full-color snapshot editor with
  mouse rectangle, numeric x/y/w/h, Full image and Refresh snapshot. Coordinates
  are top-left x/y + positive width/height, in original pixels. No persistent
  added full-color subscription: gui_roi_preview closes after snapshot/timeout/
  dialog close, independently of the regular depth/skeleton GUI previews.
- Startup-only workflow: camera may stay running; stop physical motion, disable
  motor output, finish record/export, Stop estimation → STOPPED → edit/apply →
  Start estimation. New process resets base average/filter history. External
  launched estimator must be stopped externally; direct-run default node name
  /segment_estimation_node also blocks edit/start. No live ROI mutation.
- Editor blocks active estimator, motor-enable and recording/export states;
  nested dialog Apply rechecks guards before writing. Auto-start estimation and
  Record are blocked while open. Cancel/invalid selection preserves ROI.
- Save as startup ROI defaults checked: atomically update four ROI keys in the
  resolved installed config_ROI_ref.json source, preserving symlinks/extra keys.
  Unchecked means only pending GUI-session selection. Existing user-edited ROI
  JSON was NOT changed by this implementation. At inspection it was x200,y150,
  w480,h640: invalid for 848x480 input, so operator must select valid area.
- GUI system_components forwards four readonly integer startup ROS parameters
  roi_x/roi_y/roi_width/roi_height. _launch.py defaults from ROI JSON; terminal
  overrides supported. Estimator rejects dimensions outside RGB/aligned-depth
  bounds with throttled warning and clears relevant pending frame instead of
  silently truncating NumPy crop. Recorder existing runtime-parameter snapshot
  captures actual ROI for launch-named /segment_angle_estimator.
- Validation and operator instructions: see gui_py_pkg README ROI section.
  estimation_pkg and gui_py_pkg symlink builds passed; 320 focused functional
  tests across GUI, estimator and recorder passed (legacy style tests excluded).
  Checked installed launch --show-args and offscreen synthetic-image rendering.
  New ROI modules/tests pass scoped flake8, git diff --check clean. No physical
  camera, serial device or motor was launched/restarted during this task.

## Per-session force-axis alignment for CSV (2026-09-18)

- User changes F/T sensor mounting between datasets and requested original plus
  aligned_fx/fy/fz in the same CSV row. Added GUI Force axes for CSV panel above
  Record, opt-in OFF by default; output BaseFx/Fy/Fz each select signed sensor
  input. Examples axes[+x,-y,-z] or[+x,-z,+y]. Configured rotation maps sensor→base
  column vectors. Exactly3distinct inputaxes, determinant+1 required. No arbitrary
  Euler rotation, force unit/gain/zero or action/reaction correction. Torque
  (including torque reference point) stays untouched. Mount fixed per session.
- Shared record_pkg.force_alignment helper validates configuration; GUI now has
  runtime dependency on record_pkg. RecordNode force_alignment_enabled(bool) and
  force_alignment_axes(stringarray) params default from recording.json disabled
  identity. GUI sends both with SetParametersAtomically, awaits successfulACK,
  then calls existing /data/record. Rejected/unavailable config does not silently
  start with stale settings. GUI setting+button remain locked until fresh
  lifecycle status; recorder rejects alignment changes during start/record/flush/
  export. Use one recording-control GUI; selection isn't saved across GUI restart.
- Record snapshot.force_alignment freezes canonical settings and exactmatrix
  before bag starts. status.force_alignment reports session snapshot whilelocked,
  current next-session configuration otherwise (force_alignment_scope explicit).
  Exporter uses snapshot only, never latestconfig/currentGUI/old mount notes.
- CSV on /fts_data and /fts_data_kalman_filter gains aligned_fx/fy/fz,
  aligned_frame_id(hrm_base),aligned_force_valid. Original wrench.force.x/y/z,
  torque,sourceheader/timestamps/units preserved. Invalidnumeric sample retainsraw
  butderivedblankfalse. Finitezerosare numericallyvalid, NOT proofCANconnected.
  Manifest+session_metadata.csv retain matrix/provenance. Disabled/old sessions
  withoutsnapshot remainraw-only; malformed snapshot fails beforeexportcreation.
- No liveforce/topic/controller/motor/loadcell protection changes, no additional
  fullimage subscription. CSV still generated afterRecordStop byexistingworker.
  Existing bags/CSVs notmodified; trainingtable synchronization stilldeferred.
  Restart GUI+idleDatarecorder afterbuild; olderrecorderrejectsnewparameters.
- Validation: record_pkg + gui_py_pkg built; 240 focused functional tests pass
  (legacy whole-package style tests excluded). New GUI ACK race regression makes
  post-request idle heartbeats unable to unlock before a post-ACK lifecycle.
  Domain87 force_alignment_roundtrip.py ran actualsyntheticROS recording and
  automaticCSV twice using GUI's ACK-chain helper: [+x,-y,-z] then[+x,-z,+y].
  Verified duringrecordmutationrejected,raw/Kalman values/sourceframes/torque/
  stamps preserved,alignedcolumns correct,firstsession unchanged. Artifacts:
  /tmp/hrm_force_alignment_ji43x6_u. No physicalsensor/camera/motor processes.
  Added offscreenUI tests plus actualofflinebag SHA256preservation checks.

## Serial zero-valued sensor frames (2026-09-17)

- After the port-selector update, actual host logs confirmed USB0 opened at
  921600 but startup stayed in set_zero for ~61s until operator Stop. User
  confirmed CAN module was disconnected, so firmware F/T fields were zero.
  Existing set_zero discarded an entire frame if ANY of ten fields was zero;
  read/publish thread starts only after calibration. This was not a port error.
- User approved processing zeros. Serial parser now requires exactly ten finite
  numeric fields (6 F/T + 4 loadcell) but accepts any/all zero values. Startup
  still flushes 30 reads and averages 100 accepted F/T samples; loadcell offset
  remains unchanged/zero. No timeout/reconnect or firmware changes in this step.
  No-data/malformed-only streams can still wait; a numeric zero is not no data.
- Removed unconditional finally:return 1 that hid errors/SIGINT as successful
  calibration. Interrupts/errors propagate, startup closes the opened port on
  serial failure/interruption, zero service reports its actual result.
- Without CAN, F/T topics can publish zeros while four loadcell values continue.
  This does NOT prove true zero external force or CAN health. Current protocol
  has no sensor-validity bit. Recorder freshness checks cannot distinguish those
  zeros from valid force measurements: connect/verify CAN before collecting
  training labels, then use existing Set sensor zero (F/T only) after settling.
- Validation: serial_pkg rebuilt; 91 focused tests pass (32 serial + 59 GUI).
  Real calibration/reader/publishall methods with bounded fake serial input
  verified all-zero F/T, zero loadcell channels, preserved four loadcell values,
  finite-frame checks and interruption behavior. ROS startup tests use isolated
  domain84 with device opens mocked. No actual reader restarted or device opened.

## GUI-selectable ESP32 serial port (2026-09-17)

- System panel now has ESP32 port editable selector + Refresh ports. Lists
  /dev/ttyUSB* and /dev/ttyACM* without opening devices; supports manual existing
  /dev/serial/by-id/... paths. Starts with no selection each GUI session: user
  selects before Start auto components or Serial sensors Start. No guessed
  device identity; unselected/missing serial device blocks only serial startup.
- Selected path travels through system_components.json serial_port substitution
  to serial_pkg _launch.py and readonly serial_read ROS parameter serial_port.
  Terminal launch default remains /dev/ttyUSB0; GUI requires explicit selection.
  Fixed ComponentSpec.command's blanket lowercasing: bools stay true/false, but
  case-sensitive USB/ACM/by-id paths and other strings are preserved.
- UI distinguishes current GUI-owned Launch port from pending Next Start.
  Refresh/edit never reconnects an active reader or chooses a replacement;
  Stop→STOPPED→select→Start changes the connection. External readers untouched.
  No selection persistence; no automatic reconnect/udev/permission changes.
  Existing /serial_read runtime metadata snapshot captures serial_port.
- Baud921600, timeout1s, startup F/T zero, loadcell raw values, filters, retry
  loop, motor gate, global autostart setting and other component delays remain
  unchanged. ESP32 firmware user edits were preserved.
- Validation: 66 focused GUI lifecycle/preview/record/serial-selector/backend
  tests passed; device opens and process launches mocked, isolated ROS domain84
  for serial parameter tests. No physical serial/camera/motors connected by tests.
  serial_pkg + gui_py_pkg symlink builds passed; installed launch --show-args,
  installed component command case preservation and offscreen GUI rendering
  checked. Existing launcher system.launch.py loads the updated GUI unchanged.
- Earlier cable/AprilTag segmentation discussion was advice only: estimator
  still selects largest binary component. ROI narrowing, base-seed component
  choice, tag exclusion mask and cable gap repair have NOT been implemented.

## AprilTag restoration, required data and automatic CSV (2026-09-15 night)

- Restored installed apriltag_ros (not Isaac): tag36h11 IDs0/1, frames ID0/ID1,
  existing 0.017m size. Config copied into launcher/config/apriltag.yaml for
  portability/snapshotting. Confirm physical tag measurement if labels changed.
  launcher/apriltag.launch.py now uses color/image_rect_raw + color/camera_info,
  configurable args, starts detector plus record_pkg apriltag_pose helper.
  GUI AprilTag row enters auto-component schedule at6s; global auto-start=false
  remains operator-controlled. Does not change motor startup/Enable behavior.
- New PoseStamped topics /apriltag/tag0/pose and /apriltag/tag1/pose are camera-
  optical poses; /apriltag/tag1_in_tag0/pose is inverse(camera_T_ID0)*camera_T_ID1,
  matching legacy lookup_transform('ID0','ID1'). Source-image stamp + common
  parent must match exactly. No cached/zero fallback or timer repeats when tags
  disappear. Duplicate/old TF rejected. Detector DDS endpoint change resets
  timestamp epoch. Installed Humble rclpy has no MessageInfo callbacks; use
  graph endpoint polling, not a two-argument subscription callback.
- Legacy numeric subscriptions were already covered; explicit tag pose/CSV and
  detections were missing (raw /tf already selected). New apriltag topic group
  records /detections +3poses. AprilTag YAML and both node parameters snapshot.
- Default required streams now include original RGB/aligned-depth/CameraInfo,
  motor/loadcell, raw AND Kalman F/T, wire_length, SegmentAngle, FK tip and all
  three AprilTag poses. Episodic commands/modes and mode-specific diagnostics,
  unused AI external force remain optional. Required small telemetry must be
  recently progressing before Record; image publishers+CameraInfo checked live,
  NOT Python copies of full images. RGB/depth persistence is audited after Stop.
- Live missing/stale data (>2s default required_max_gap_sec) produces DATA WARNING
  and metadata events; recording continues, motors untouched. Startup readiness
  timeout10s stops only recording. Publisher GID changes reset telemetry epoch.
  Closed SQLite bag audit checks actualcounts/internal/leading/trailing receive
  gaps in recording-ready→FIRST Stop ROS-time window (not startup/flush delays).
  Live source-progress gaps also prevent a final healthy-quality label even if
  bag receipt counts look good. complete is a scoped continuity check, NOT
  sensor validity, exact synchronization or frame-loss-free hardware proof.
- GUI Record Stop: stopping(flush)→exporting(audit+CSV)→stopped. New Record is
  disabled while exporting; incomplete is amber, conversion failure failed but
  original bag preserved. auto_export_csv=true default; false still audits.
  All selected topics share one bag session, SQLite splits at2GiB, not one file
  per topic. csv/manifest.json maps separate per-topic CSV filenames (hash suffix).
  Arrays use lossless JSON cells; full4motor/4LC/18angle entries retained. Pose
  includes quaternion+derivedRPY(rad); TF one transform per row. Original source
  and bagreceive ns kept separate; images/depth/cloud payloads stay in bag.
- Manual export: ros2 run record_pkg export_csv record/<session> [--output NEW_DIR].
  No ROS node/replay/publications, no overwrite; splitbags handled by metadata.
  session_metadata.csv holds export-start metadata snapshot; session.json is
  authoritative final lifecycle/dataquality. postprocess.json/export.log hold
  automatic worker outcome. Stopping GUI/component before export completes
  preserves/flushes bag but CSV may be pending_or_interrupted; retry into a NEW
  output directory. No destructive cleanup of previous recordings.
- Validation: record_pkg/gui_py_pkg/launcher built; 90 focused recorder/metadata/
  geometry/health/audit/export/finalizer/GUI unit tests passed. Actual installed detector
  syntheticimage test domain79 passed with BEST_EFFORT input,17mm scale,source
  stamps, same-image relativepose, duplicates/mismatches and detectorrestart.
  Domain80 actual recorder rejects publisher-without-samples, warns on stalled
  required pose while still recording, writes bag+CSV and flags incomplete;
  unique published source stamps only, no stale/zero rows. Domain76 updated
  metadata_roundtrip passed (compiled readonly constants still correct), plus
  rosbag_roundtrip passed with233 RGB/depth/angle frames,775 motor/force/LC,
  no interior sourceframe drops, originalpayload/stamps, automatic CSV complete,
  and shutdown flush with pending export. No physical camera/TCP/motor launched.
- Still deferred: image-time synchronized training-table construction, offline
  angle reconstruction/label frame conversion, training models. CSV is lossless
  per-topic export, not a fabricated synchronized replacement for legacy CSV.
  See src/record_pkg/README.md and src/gui_py_pkg/README.md for tomorrow's workflow.

## Recording hardware/config metadata (2026-09-15 evening)

- User requested hardware constants/settings in recorded metadata. `session.json`
  schema 2 now includes session_id, timezone-aware local start and matching UTC;
  folder naming remains `YYYYMMDD_HHMMSS_microseconds` in PC local time. Inner
  files: `bag/bag_0.db3` (split numbering), bag metadata.yaml, session.json,
  recording_config.json, qos_overrides.yaml, recorder.log.
- ControlNode exposes read-only `hardware.*` parameters from the compiled
  hw_definition.hpp, ignoring CLI/YAML overrides. OP_MODE=8/CSP, geometry,
  counts, gear/encoder definitions and limits included; **NOT drive readback**.
  No control math, safety policy or motor output behavior changed in this step.
- Recorder asynchronously lists/gets scoped node parameters at each Record start
  (metadata_nodes/metadata_timeout_sec, default 3s). Bag ingestion begins first;
  recording-ready waits for queries to finish/time out as well as subscriptions.
  Saves typed values, request/completion times, unavailable/timeout/ambiguous/
  cancelled status. Missing optional metadata is partial, not fatal. No guessed
  runtime fallback from a source file. Stop/shutdown cancels pending queries.
- Snapshot includes exact text/SHA-256/path/time of installed hw_definition.hpp,
  control_parameters.hpp, estimation config/ROI and GUI component config;
  original package_configs/last-observed filter snapshot retained. Files/notes
  are explicitly distinct from runtime parameter values. Config edited after
  node start can differ from loaded settings. Parameter replies are not atomic
  across nodes and do not expose all internal filter/controller state.
- `/control_mode` and GUI op_mode are distinct from compiled OP_MODE. Initial
  public ROS settings are queried even if no volatile topic event was received;
  parameter_events/control_mode/data_filter_setting still record later changes.
  Metadata target names follow standard GUI launch; adjust list for direct-node
  names/namespaces. Rebuild/restart robot_control and recorder to get compiled
  constants from the new binary; older controller reports unavailable.
- Validation: robot_control_pkg + record_pkg built (existing C++ warnings);
  36 focused recorder/metadata/GUI unit tests and 17 FK/IK/PD gtests pass. Isolated domain 76 real bag
  metadata_roundtrip passed twice: two sequential sessions with changed parameter,
  ignored hardware CLI override and rejected runtime write, real compiled
  19 segments/18 joints/82.27mm/OP_MODE=8, overridden PD gain, JSON byte arrays,
  missing-node timeout, immediate-stop partial snapshots. Zero actuator messages;
  output=false, remapped motor topic, no camera/TCP/physical motor processes.
  Existing rosbag_roundtrip also re-passed after this change: 244 synthetic
  RGB/depth/angle frames, preserved stamps/payloads, zero interior source-frame
  drops, late-join static TF and active-recording launch shutdown flush verified.
- See src/record_pkg/README.md for field descriptions and tests. CSV export was
  subsequently implemented in the section above; never replay recorded
  motor_command into the hardware domain.

## Image-paced position PD and mode-entry reset (2026-09-15)

- Implemented user's approval: no repeated accumulation of the full PID output,
  lower gains with I=0, one update per new source-image feedback, current-position
  goal/PID reset only on mode entry. Y→tilt, Z→pan, passive X and FK/DH/base
  rotations unchanged. Cross-axis compensation intentionally deferred.
- Law: `q_target = q_entry + Kp*(goal-actual) - Kd*filtered(actual_velocity)`.
  q_entry is fixed until re-entry (sum of active pan/tilt joints, NOT tip Euler
  orientation). This is an absolute PD correction about the entry point, not
  `q_previous += full_PD` or `q_measured + full_PD`. I=0 means a steady-state
  position error can remain; no claim of exact goal tracking/stability yet.
- Defaults BOTH axes: P=5 rad/m, I=0, D=0.05 rad*s/m; measurement derivative
  low-pass tau=0.05 s (no setpoint derivative kick); angular slew=5 deg/s per
  aggregate cable-IK axis; existing aggregate angle limit ±60 deg. These are
  bring-up settings, NOT validated physical safety limits. ROS startup/runtime
  pan AND tilt overrides are applied; nonzero I/invalid gains are rejected.
- Position/Admittance worker waits on new accepted SegmentAngle source stamps,
  uses their actual dt, does not repeat cached/duplicate/reordered frames. Checks
  hrm_base, all eight length-18 finite arrays, motor/loadcell length four and
  receipt/source age (default maximum 0.25 s; future tolerance 0.05 s).
  Force has only a receipt-age check because its legacy Vector3 lacks a header.
- Follow-up user correction: camera-only gaps/stale/invalid frames now skip
  control updates with WAITING, not a latched fault. Keep the pending goal,
  entry angles/encoder reference and last motor target; automatically resume
  on the first fresh valid image without mode re-entry or an extra hold command.
  A single small frame delay did not exceed the old 0.25 s threshold either.
  After source gaps >0.25 s reset only PD derivative history, not the goal/bias;
  diagnostics keep actual dt, angular slew allowance uses min(dt,0.25 s), and
  legacy admittance skips force integration across the long gap (no catch-up).
  This also permits consecutive fresh images at less than 4 Hz. Images older
  than the configured age limit are still not used for control.
- Mode/enable transitions require a new post-transition source frame; set goal
  to fresh FK tip and reset PD/admittance states. Preserve current encoder counts
  via `motor_entry + scale*(IK(q)-IK(q_entry))`, so first output holds the exact
  four motor positions rather than returning to encoder zero. Cancel old demo
  timers on transitions. Setting the same mode does not reset the running goal.
- Motor/loadcell/force freshness failures, tension, invalid (nonpositive/nonfinite)
  dt and target failures after entry still latch FAULT; valid motor feedback
  permits one current-position software hold. These guards run even while waiting
  for an image. For these non-camera faults leave/re-enter the mode or toggle output. Re-enabling
  captures a NEW goal/current position; old target is discarded. Mode exit/output
  disable also requests a hold before gating new commands. Missing/stale/out-of-
  range motor feedback cannot safely produce a hold: driver watchdog/E-stop is
  still required. New outgoing position targets checked against ±motor limit.
- Dry-run preview `/position/target_motor_command` (MotorCommand) and latched
  `/position/control_status` (String: WAITING/READY/ACTIVE/FAULT/INACTIVE) added.
  Recorder motor/control group includes both. Existing PositionControl gains and
  theta arrays are still pan-only legacy fields; full measured angles remain in
  SegmentAngle. Diagnostics now carry source image header in hrm_base.
- Tests: robot_control_pkg rebuilt after camera-policy correction; 12 PD + 5 FK/IK
  gtests pass (including long-gap recovery and bounded slew). record_pkg build
  and 20 focused recorder/GUI tests passed in the preceding implementation.
  `test/position_frame_smoke.py` uses isolated domain 78/localhost, fake feedback,
  output=false AND remapped actuator topic. Verified empty-sensor startup, nonzero
  entry without motor jump, 10→20 Hz/source dt, repeated-error no drift, duplicate,
  NaN/wrong-frame/stale-angle rejection, empty loadcell, same-mode no reset and
  re-entry reset. Updated repeat verified duplicate/missing (0.7 s)/slow (0.35 s
  intervals) camera input automatically resumes with the same goal/reference,
  without extra commands during the gap. Motor/loadcell faults still latch even
  during camera gaps; tension trip and pan/tilt startup/runtime gains verified.
  Zero actuator messages observed. The test now captures entry diagnostics before
  the mode-change request because feedback can arrive before the service reply.
  No camera/TCP/physical motor tests and no running GUI restart performed.
- Remaining: admittance force frame/unit/sign/Y-only dynamics still old and NOT
  ready; dynamics/TCP safety not redesigned. Cable signs, attainable workspace,
  physical gains/slew/tension/travel limits, latency/filter lag and driver stop
  must be verified before enabling feedback control. Switching from entry-offset
  Position commands back to absolute IK requires the established motor zero and
  pretension reference; do not assume all command modes share a new zero.
- DynamicsController is still compiled and instantiated as ControlNode::HRM_controller_.
  Its worker is created at node startup but compute() runs only in control_mode=2,
  not in Position=3 or Admittance=4. It still uses legacy PID compute_output().
  Do not remove that function under the assumption that it is globally unused.
- Current details/reproduction commands: `docs/CONTROL_READINESS.md`. The audit
  below is historical: its crash, old gains and fixed-dt accumulation describe
  the code BEFORE this update, not the current position path.

## Position/admittance audit and 3D recording (2026-09-14 evening)

- **NOT READY for physical position/admittance closed-loop use.** Full findings,
  source locations, numerical results, contracts and next-test checklist are in
  `docs/CONTROL_READINESS.md`. Controller runtime code was not modified in this
  audit. No hardware output was enabled or motor command sent.
- Confirmed current x_actual is filtered-angle DH FK tip in hrm_base/metres,
  not a TF lookup or directly measured skeleton endpoint. Y→tilt (q1 local Z),
  Z→pan (q2), X passive. Fixed proximal os + 18 joints = 19 segments/82.27 mm.
- Standalone actual C++ math: 1 mm error gives delta 3.25017 rad and immediately
  hits 60° IK clamp; stale 0.1 mm error repeated ten times reaches 31.5652°.
  Admittance responds only on Y; +0.1 N on Z yields zero. Hardcoded desired
  0.02 N yields +1 mm Y offset at zero external force. PID dt=0 yields inf.
- Sensor-free Position startup in isolated ROS_DOMAIN_ID=77, output=false,
  reproduced SIGSEGV/exit139; gdb stack is the position/admittance thread.
  loadcell stress indexing precedes input validation/output gate. Do not treat
  dry-run as protection against this software crash. Existing FK/IK 5 gtests pass.
- Other blockers: external_force subscriber drops Z, old `(Fy,-Fx,0)*0.001`
  remapping remains; singular Y-only 6x6 admittance; no sensor age/watchdog,
  integrator/output-rate limits or safe mode/Enable reset; cached feedback runs
  every fixed dt regardless of new data; only pan PID runtime tuning. Velocity
  uses Y error only but has minimum 10 (NOT a zero-speed pure-Z bug).
- Audit also flags asymmetric TCP position limit, partial TCP frames, and
  last-target retransmission: stopping ROS publication is not physical E-stop.
- record_pkg now wraps the C++ rosbag engine behind unchanged `/data/record`.
  Original RGB/depth/CameraInfo, raw AND Kalman F/T, offsets, four motor/loadcell
  channels, cable commands, complete SegmentAngle, TF, controller diagnostics
  and optional visualizations are retained with source headers. Default D405
  RGB is `/camera/camera/color/image_rect_raw`; cable preview is
  `/kinematics/target_wire_length`. Correct group/required names if remapped.
- Configure `src/record_pkg/config/recording.json`; GUI Record component is
  still manual/off by default. Recording produces `record/<session>/bag` plus
  metadata/config/QoS/logs. Missing required publishers refuse start. Wait for
  recording readiness and then stopped/flush completion. Raw label coordinates
  are NOT rotated on capture. See `src/record_pkg/README.md`.
- Previous CSV recorder preserved at `src/record_pkg/legacy/record_node_2d.py`;
  no existing datasets removed. Asked whether to drop obsolete 2D derived CSV/
  AprilTag processing and use rosbag + later export; no reply at implementation
  time. Retained raw/Kalman, dynamics diagnostics and visualization selections;
  legacy derived algorithms remain available as reference. Dedicated offline
  dataset CSV/image export was not implemented at this audit; see the newer
  AprilTag/automatic CSV section above for the subsequent implementation.
- GUI record button now follows service result and `/data/record_status`
  heartbeat: starting/recording/stopping/stopped/failed/stale, with failed-start
  explanations and no premature success. Other motor/layout behavior unchanged.
- record_pkg/gui_py_pkg built successfully; 54 focused capture/GUI tests pass.
  Real isolated rosbag round trips at 848x480 RGB/depth 30 Hz + sensors 100 Hz
  recovered 157 and 159 frames per stream over ~5 s sessions, with no interior
  source-frame gaps; payloads/source timestamps, 18-angle arrays, 4 channels,
  distinct raw/Kalman force and pre-existing static TF preserved. These are
  synthetic transport/storage tests, NOT integrated live-hardware guarantees.
  Final repeat recovered 157 frames and also verified that stopping the owning
  launch during a second active recording flushes/closes that bag successfully.
- User reports legacy training used `fx`. Repository history distinguishes
  `fx` (fts_data, already zeroed/optional LPF/MAF) from `fx_kalman` (separate
  Kalman topic). Preserve both for now. Sensor-to-base force mapping supplied
  by user: Fb=[Fsx,-Fsz,Fsy], fixed relative mounting only; offline labels should
  be transformed and unit/sign validated before Base-frame inference/control.

## Four-channel loadcell GUI (2026-09-14)

- Serial already publishes four values in `/loadcell_state.stress`; only the
  GUI still created/updated two rows. GUI now declares `num_loadcells=4`
  independently of motor count, with the same launch default, and displays
  all four channels in received order. Missing values are cleared to `—`.
- Sensor zero remains below the loadcell rows. No serial, motor command,
  force/torque plot, calibration or hardware connection behavior was changed.
- 34 focused GUI tests passed, including four-channel values and partial/empty
  messages clearing old values. GUI package rebuilt; operator GUI was not
  restarted during the edit.

## GUI Stop/External lifecycle fix (2026-09-14)

- A user Stop really terminated robot_control (PID 56281, SIGINT exit -2),
  but the old GUI forgot its stop history once the launch parent exited and
  interpreted any remaining ROS graph name as External.
- `system_manager.py` now retains the stop/exit history and owned process-group
  identity. Child processes outliving their launch parent remain owned and
  stoppable, never relabelled as External. No external targets are killed by
  node name. Partial external component presence also blocks duplicate starts.
- Normal stop is a single SIGINT to the ROS launch parent, with stdin DEVNULL
  guaranteeing noninteractive launch forwarding. This avoids sending SIGINT
  both directly to children and again via launch. Repeated Stop is idempotent.
  The GUI waits 12 s before fallback SIGTERM to its own group, allowing launch's
  native escalation to finish first. Stop initiation time is retained.
- UI: STOPPING while owned processes remain; STOPPED* / ROS cleanup… if the
  group has exited but names remain in the graph; STOPPED / Start after graph
  clearance. Restart is blocked during ambiguous graph cleanup. A new external
  node after observed clearance is still protected. Unexpected failures remain
  failures, while operator-requested signal exits count as stopped.
- Built gui_py_pkg. 31 focused tests passed, including duplicate signal
  prevention, stop-history persistence, stale graph/restart guards, surviving
  children, external protection and actual GUI status/button states.
- Three real dry-run robot_control start/stop cycles in ROS_DOMAIN_ID=74
  each finished in ~0.15 s, exit code 0, STOPPING → STOPPED, without External.
  Shutdown logs include `ROS node Shutdown` and `process has finished cleanly`.
  Test launches were cleaned up; no active hardware processes were stopped.

## Motor GUI layout and image previews (2026-09-14)

- Read-only diagnosis of Direct motion: TCP feedback had four positions and
  the motor_command subscriber was connected, but robot_control's
  `motor_output_enabled` was false. Direct `1000` means current encoder
  position +1000 counts; no test movement or gate change was performed.
- Moved the unchanged physical-output checkbox/acknowledgement to the top of
  Motor control. Startup remains OFF and operator confirmation is required.
  Motor control now takes roughly 60% of the window; System remains above
  Sensors on the left. Mode selections are horizontal to leave room for images.
- Moved **Set sensor zero** under the raw sensor values. It still calls
  `/serial_data/set_zero`, not a motor zero/homing service. The current serial
  implementation averages force/torque data; loadcell averaging remains
  commented out (no serial behavior change in this task).
- Added `gui_py_pkg/image_preview.py`: aligned depth and skeleton side by side
  under the motor commands, independently updated. Depth JET display uses
  near=red/far=blue, invalid=black, adjustable fixed 0.07–0.6 m range.
  16UC1 millimetres and 32FC1 metres are supported; raw depth is untouched.
- Image-only node `gui_image_preview` has its own executor/thread, sensor QoS
  (BEST_EFFORT, KEEP_LAST(1), VOLATILE), one latest-message slot per topic,
  max-640px display width and a 34 ms Qt refresh timer. Widget updates are
  confined to Qt; QImage owns copied pixels; missing/stale streams are labelled.
  Preview worker is joined before the GUI shuts down ROS.
- Built gui_py_pkg; 19 focused tests passed, including image formats/units,
  invalid depth, padded/big-endian depth, latest-only buffering, stale states,
  image-buffer lifetime and relocated buttons. New preview code passes targeted
  flake8/pydocstyle checks. Existing workspace-wide flake8/pep257 tests still
  fail on legacy/generated sources (not treated as a clean full-suite result).
- Isolated ROS_DOMAIN_ID=74 GUI test: 30 Hz synthetic image inputs plus 500 Hz
  synthetic motor feedback, ~27 Hz displayed, Qt heartbeat p95 ~22 ms and max
  ~28 ms. Live read-only preview check: 391 depth / 371 skeleton frames displayed
  in ~14 s. These are UI refresh observations, not end-to-end camera benchmarks.
  Test processes were closed; existing hardware/GUI sessions were not restarted.
- `launcher/system.launch.py` currently defaults auto_start_components=false
  due to an operator change; it was preserved. Restart GUI only after safely
  stopping physical motion. Detailed operator notes: `src/gui_py_pkg/README.md`.

## Camera/estimation real-time fix (2026-09-13 evening)

- Operator instructions and measured limits: `docs/REALTIME_ESTIMATION.md`.
- The earlier claim that 81% estimator CPU usage itself proves the camera
  bottleneck was too strong. Earlier measurements also included USB/device
  resets. A controlled synthetic A/B now demonstrates a transport bottleneck:
  default 512 KiB Fast DDS SHM is smaller than RGB (1.16 MiB) or depth
  (0.78 MiB). The same optimized estimator with 512 KiB SHM lost images;
  64 MiB SHM restored all streams to ~30 Hz.
- `estimation_pkg/config/fastdds_images.xml` supplies SHM 64 MiB / max message
  4 MiB plus UDPv4. `launcher/cuda_realsense.launch.py` applies it to the CUDA
  camera, so both GUI and `scripts/run_realsense_cuda.sh` share the fix.
  `estimation_pkg/runtime.py` applies it before rclpy.init for direct run and
  launch. Explicit operator DDS profiles/other RMW selections are preserved.
  Restart BOTH camera and estimator to apply it; no global OS changes needed.
- Lee skeletonization is restricted to the nonzero body bounding box, preserving
  exact original ROI pixels. The 2D extrapolation/body intersection, depth
  deprojection, base orientation, hardware scaling and DH/filter math remain.
- Reconstruction no longer holds the visualization handoff lock. One complete
  result snapshot replaces the previous pending result; crop publication is
  handled outside DDS callbacks. No old frame is republished just to raise Hz.
- RViz ARROWs retain existing types/namespaces/dimensions but update stable
  `(ns,id)` objects instead of DELETEALL/recreate on every frame. Missing IDs
  are deleted explicitly; only the initial publication uses DELETEALL.
- New config: `performance.blas_num_threads=1`, `max_depth_age_sec=0.1`.
  The latter rejects color/depth pairs more than 100 ms apart (0 disables);
  it is a stale-depth guard, not exact RGB-D synchronization. Invalid/empty body
  frames warn at most every 2 s and do not emit fresh angles. Debug rate logs
  run even during failures; timing/rate logs still default OFF.
- `image_rate_probe/pipeline_rate_probe` now measures color/depth/crop/skeleton/
  angles/markers rates, source age p50/p95 and repeated/backwards/missing stamps.
- Verified: 90 s / 2700 synthetic moving frames with estimator + dry-run FK +
  all probe image/marker subscribers, steady 30 Hz outputs, zero repeated or
  backwards timestamps and zero estimated missing periods in steady windows.
  Source-to-angle p95 by 5 s window was 20.2–36.4 ms. Real D405 + offscreen GUI
  + estimator + robot_control + existing RViz also sustained 30 Hz RGB/aligned
  depth/crop. REAL GEOMETRY LIMIT: the camera ROI was dark (maximum 19 vs
  threshold 120), so real-HRM angle success/accuracy needs illuminated testing
  tomorrow. Motor output stayed disabled; no TCP bridge was started in tests.
- Reproduce the isolated throughput test with
  `python3 scripts/benchmark_estimation_pipeline.py --duration 90` after sourcing
  the workspace. It uses test ROS domain 73 and cleans up its own children.
- Estimation/camera autostart remains enabled. The old recorder is now manual
  pending migration to final 3D dataset topics, as previously deferred. GUI
  close stops Qt/plot timers before destroying the ROS node to fix a teardown
  InvalidHandle exception discovered in the integration test.
- Built estimation_pkg, gui_py_pkg, launcher, image_rate_probe successfully;
  16 focused tests passed (including GUI process-manager tests). Geometry,
  skeleton equivalence, snapshot, marker and DDS override regression
  checks are in `test_realtime_pipeline.py` and `test_hardware_layout.py`.

## GUI-managed system startup (2026-09-13)

- `launcher/system.launch.py` now starts only `gui_py_pkg`; the GUI owns the
  child launch processes and provides per-component Start/Stop controls.
- Ordered defaults live in
  `src/gui_py_pkg/config/system_components.json`: CUDA camera (0 s), serial
  (1 s), TCP motor bridge (2 s), robot control (3 s), estimation (4 s).
  Recorder remains optional/manual (5s). AprilTag was subsequently added to the
  automatic schedule at6s (2026-09-15); global auto-start toggle remains unchanged.
- `daq_pkg` and `MasterMACS_pkg` are intentionally excluded.
- Status uses both the managed process and expected ROS graph node. An
  externally started node is shown as EXTERNAL and is never killed by GUI.
- Physical motor output always starts disabled. The Motor control safety switch
  updates `/robot_control`'s `motor_output_enabled` parameter; if control is
  stopped, its selection is passed as the next launch argument.
- The motor UI is parameterized for four motors in East, West, South, North
  order and displays the four-value IK preview.
- The black/frozen GUI seen without motor or serial data was caused by calling
  `rclpy.spin_once()` with its blocking default inside the Qt event loop. It is
  now called with `timeout_sec=0.0`; missing streams are shown as red
  `no recent data` labels instead of freezing repainting.
- The GUI is one maximized window split into two columns. The left column has
  System above Sensors, while Motor control occupies the full-height right
  column. The Sensors panel has raw values on its left and a wide plot on its
  right.

## Purpose

This document carries project decisions and implementation state between Codex
sessions and computers. Read it together with the current source, `git status`,
`git diff`, and recent `git log`; the repository is the source of truth when
this document and the code differ.

## Robot and control model

- The continuum section has two controlled bending DOFs: pan and tilt.
- East/West cables form the antagonistic pair for pan.
- South/North cables form the antagonistic pair for tilt.
- Hardware inspection corrected the physical order on 2026-09-13: the first
  revolute joint is driven by the South/North pair and is **tilt**, followed by
  the East/West **pan** joint. The 18 one-axis joints therefore alternate as
  `q1=tilt, q2=pan, ..., q17=tilt, q18=pan` (zero-based even indices are tilt).
  The estimator and FK were converted to this tilt-first convention on
  2026-09-13.
- The physical centerline contains one fixed proximal segment `os` followed by
  18 moving-link directions, for 19 geometric segments and 20 boundary points.
  The kinematic chain has 18 revolute joints (nine pan/tilt pairs); `q1` acts
  after the fixed `os` segment.
- `hrm_base` is the authoritative Cartesian frame. X is the tool's axial
  direction and is calculated but is not a position-controller input.
- `hrm_base` is also the D-H Base, with no preliminary rotation. q1 is named
  tilt, rotates about Base +Z, and positive q1 bends the axial +X direction
  toward Base +Y. q2 is named pan and is the next orthogonal one-axis joint.
- The position feedback dimensions are Base Y for tilt and Base Z for pan in
  the straight configuration.
- There is currently no insertion-axis control.
- `PositionController` independently uses Y and Z errors:
  - `x_desired(1) - x_actual(1)` -> tilt PID -> `del_theta_tilt_`
  - `x_desired(2) - x_actual(2)` -> pan PID -> `del_theta_pan_`

## Segment-angle interface decision

Pan/tilt angles and velocities are grouped into one synchronized custom message
instead of separate topics.

- Message: `custom_interfaces/msg/SegmentAngle.msg`
- Topic: `estimated_segment_angle`
- Every array is ordered from base to end-effector and must contain
  `NUM_OF_BENDING_JOINTS` elements.
- Units:
  - angle: rad
  - angular velocity: rad/s

Message fields:

```text
std_msgs/Header header
float64[] pan_relative
float64[] pan_absolute
float64[] tilt_relative
float64[] tilt_absolute
float64[] pan_angular_velocity_relative
float64[] pan_angular_velocity_absolute
float64[] tilt_angular_velocity_relative
float64[] tilt_angular_velocity_absolute
```

`ControlNode` initializes and validates all arrays and receives them through a
single subscriber. The RBSC estimator now publishes this message after each
successful 3D reconstruction. Every array has 18 entries. Relative arrays put
the projected DH angle in the active alternating axis and zero in the inactive
axis. Absolute pan/tilt are the Base-frame azimuth/elevation of each projected
outgoing segment. With the joint Kalman filter enabled, relative joint angular
velocities are the filter's `q_dot` states; absolute direction-angle velocities
still use wrapped temporal finite differences. With Kalman disabled, all four
velocity arrays use wrapped temporal finite differences. The first valid
message has zero velocities.

## Estimation repository layout

- The runtime ROS package was copied from
  `src/LSTM-force-estimation/src/estimation_pkg` to the first-class package
  `src/estimation_pkg` in this repository.
- Develop runtime segment-angle estimation, external-force inference, ROS
  publishers, and deployment-model loading in `src/estimation_pkg` from now on.
- Keep datasets, training scripts, multi-model experiments, comparisons, and
  research checkpoints in the `src/LSTM-force-estimation` submodule.
- `src/LSTM-force-estimation/COLCON_IGNORE` prevents colcon from discovering
  the submodule's duplicate ROS packages.
- `COLCON_IGNORE` is currently an uncommitted file inside the submodule. To
  preserve it on another computer, it must eventually be committed and pushed
  in the submodule repository, followed by committing the updated submodule
  pointer in this parent repository.

## Current forward-kinematics state

- Hardware counts are now separated into nine pan/tilt joint pairs for the
  cable IK and eighteen alternating one-axis bending joints for segment-angle
  input and D-H forward kinematics.
- The confirmed modified-DH convention matches the estimator:
  `^(i-1)T_i = Rx(alpha_(i-1)) Tx(r_(i-1)) Rz(q_i)`, with `d_i=0` and
  `alpha=[0,+90,-90,+90,...] deg`. The recursion starts at identity in the
  fixed `hrm_base`; there is no initial `Rx(+90 deg)`.
- `q1=tilt_relative[0]`, `q2=pan_relative[1]`, and so on through q18. Inactive
  entries in the two synchronized arrays remain zero but are selected by joint
  parity rather than added.
- Confirmed centerline distances are `os=l=le=4.33 mm`. The 18 joint boundary
  transforms start 4.33 mm from `hrm_base`; the separate tip is another
  4.33 mm after q18. The zero-angle Base-to-tip distance is therefore
  `19 * 4.33 mm = 82.27 mm`.
- `computeBaseToJointsTransformationMatrices()` now returns all 18 Base-to-
  joint transforms. `computeJointPositions()` returns their `Eigen::Vector3d`
  translations, and the separate end-effector helpers apply `le` after q18.
  Consequently `hrm_fk_joint_01` is identity at `q1=0` and its first
  variable rotation is q1 tilt about Base `+Z`; `hrm_base` itself never
  rotates.
- `ControlNode` now passes both relative pan and tilt arrays to FK and uses the
  resulting 3D tip position for `x_actual`.
- The estimator now also broadcasts `estimated_segment_center_01` through
  `_19` directly under `hrm_base`. Their translations are the 19 measured
  per-frame 3D curve center samples. Center 1 is the fixed `os` center and uses
  the Base orientation; centers 2 through 19 use the 18 filtered sequential
  D-H joint rotations. Center translations have spatial smoothing from depth
  median and curve fitting but no separate temporal position filter.
- On every valid `estimated_segment_angle`, `ControlNode` broadcasts the 18
  model-derived joint-boundary frames `hrm_fk_joint_01` through `_18` plus
  `hrm_fk_tip`, all directly parented to the message frame (normally
  `hrm_base`) and stamped with the source image timestamp. These are model/FK
  frames, so they belong to `robot_control_pkg`; the control calculation does
  not perform TF lookups.

Current interface:

```cpp
Eigen::Matrix4d computeTransformationMatrix(
  const double& joint_angle,
  const double& twist_angle,
  const double& previous_link_length) const;

std::vector<Eigen::Matrix4d> computeBaseToJointsTransformationMatrices(
  const std::vector<double>& pan_angles,
  const std::vector<double>& tilt_angles) const;
```

## Agreed 3D curve-reconstruction stage

The implementation specification is preserved at:

```text
src/estimation_pkg/docs/3d_curve_reconstruction.md
```

The implemented scope is ordered depth-based 3D skeleton points,
cumulative-distance parameter `s`, independent quartic fits `x(s)`, `y(s)`,
`z(s)`, dense fitted-curve sampling, fitted-curve arc-length reconstruction,
19 hardware-length intervals, 20 boundary points, and 19 straight segment
directions. Direction 0 is the fixed `os` segment; a preliminary sequential DH
projection uses directions 1 through 18 to estimate and publish 18 alternating
pan/tilt joint angles. Real-camera sign, noise, and dynamics validation remain
outstanding.

Preserve the existing 2D centerline extrapolation before depth deprojection:
initial skeleton -> 2D fit/extrapolation -> conversion back to pixels -> AND
with `body_image` -> `extended_skeleton`. Apply aligned depth to the final
`extended_skeleton` pixels, not only to the initial skeleton pixels.

Current implementation details:

- `segment_angle_estimation.py` subscribes to color, aligned depth, and color
  `CameraInfo`; topic names and depth scale are ROS parameters.
- `estimation_pkg/launch/_launch.py` starts only `segment_angle_estimator`;
  `external_force_estimator` and the delayed RQT GUI are intentionally excluded.
- `postprocess.py` retains fitted-curve order through pixel conversion and the
  body-mask intersection instead of reconstructing points with scan-ordered
  `np.where`.
- Valid pixels are first pinhole-deprojected into the optical camera frame: X
  right, Y down, Z forward/depth. The first valid base-side point is then
  subtracted. The fallback orientation describes the base pose in camera axes
  as `R_camera_from_base = Ry(-90 deg) @ Rz(-45 deg)`; its inverse/transpose is
  applied to express relative points in the base frame.
- `transform_camera_points_to_base()` accepts a runtime 3x3 rotation or 4x4
  Camera-from-Base transform. Only rotation is used; base translation remains
  estimated from the first ordered 3D skeleton point. The ROS node does not yet
  look up an external live rotation for this input, so it currently uses the
  configured fallback angles `base_in_camera_rotation_y_deg` and
  `base_in_camera_rotation_z_deg`.
- The optional Base-origin filter initially averages 15 camera-frame Base
  samples, then fixes that average. Every current camera point subtracts this
  same fixed origin before rotation into `hrm_base`; it is not an offset applied
  only to the first point. During warm-up, the running mean is used. The raw
  first Base-frame point may therefore retain a small diagnostic residual after
  the origin is fixed, while the fitted curve is explicitly anchored at
  `(0, 0, 0)`.
- Base-frame coordinates are isotropically normalized by the fixed hardware
  length `os + 17*l + le` before 3D fitting. With the current equal lengths,
  this is `19 * 4.33 mm = 82.27 mm`. The configured base frame ID is
  `hrm_base`.
- After fitting, the Base-anchored curve is isotropically scaled once so its
  arc length is exactly 82.27 mm. This prevents depth/end-point length error
  from accumulating between camera reconstruction and fixed-length FK.
  `fitted_curve_length_before_hardware_scaling_3d` and
  `hardware_length_scale` retain the measured length and correction factor.
- The parametric 3D quartic is fitted without a constant term for each axis, so
  the reconstructed curve itself is constrained to `r(0) = (0, 0, 0)`.
- `points_xyz_camera` retains the deprojected camera coordinates;
  `points_xyz_base` and the compatibility alias `points_xyz` contain base-frame
  coordinates used by reconstruction.
- Runtime outputs are `points_xyz`, `segment_points_xyz` with shape `(20, 3)`,
  `segment_directions_xyz` with shape `(19, 3)`, and analytic fitted-curve
  `segment_tangents_xyz` with shape `(20, 3)`. Each hardware-length segment is
  also
  sampled at its fitted-curve midpoint, producing
  `segment_center_points_xyz` and `segment_center_tangents_xyz`, both with
  shape `(19, 3)`.
- All 19 raw `segment_directions_xyz` are preserved. Direction 0 represents
  fixed `os`; the sequential modified-DH projection uses directions 1 through
  18 with `alpha=[0,+90,-90,+90,...] deg` and produces
  `segment_directions_projected_xyz`, `dh_joint_axes_xyz`, signed
  `segment_projection_residuals`, and preliminary
  `dh_joint_angles_projected_{rad,degree}`. The orientation recursion is
  `R_B_i = R_B_(i-1) Rx(alpha_(i-1)) Rz(q_i)`, initialized with the identity
  rotation of fixed `hrm_base`, with joint 1 as tilt about Base +Z. These
  preliminary angles remain available as raw measurements. An optional
  constant-velocity Kalman filter maintains one `[q, q_dot]` state per joint;
  its filtered angles feed `estimated_segment_angle` and regenerate
  `segment_directions_filtered_xyz`. These still require real-camera and robot
  sign/convention validation before control use.
- A configurable spatial neighborhood median is applied to depth at each
  skeleton pixel before pinhole deprojection. Zero/non-finite depth is ignored
  inside the window and still rejected if the final sample is invalid.
- All stabilization stages are independent in `config.json` under `filters`:
  `base_origin_initial_average`, `depth_neighborhood_median`, and
  `joint_kalman`. Set each stage's `enabled` value independently. Raw depth,
  raw segment directions, and projection-only joint angles remain stored for
  diagnosis.

Visualization topics are separated as follows:

- `estimated_segment_crop_image`: unmodified ROI in the camera's native RGB/BGR
  encoding
- `estimated_segment_body_binary_image`: mono8 body mask
- `estimated_segment_skeleton_image`: ROI with valid ordered skeleton overlay
- `estimated_segment_centerline_points`: ordered base-frame PointCloud2
- `estimated_segment_reconstruction_markers`: fitted curve, 19 reconstructed
  segments, 20 boundary points/tangents, and 19 segment-center
  points/tangents for RViz2. Center points use the
  `segment_center_points` namespace and center arrows use
  `segment_center_tangents`. Raw segment directions are gray vectors in
  `raw_segment_directions`; projected directions are orange vectors in
  `projected_segment_directions`; Kalman-filtered DH directions are green
  vectors in `filtered_segment_directions`, so all layers can be toggled
  separately. Each vector is now an individual `ARROW` marker because ROS has
  no `ARROW_LIST` type. Arrow shaft/head dimensions are 0.3/0.6 mm with a
  0.8 mm head length. This restores visible arrowheads but increases each
  MarkerArray by roughly 100 Marker objects, so recheck its live rate if RViz
  performance regresses. MarkerArray publication is not rate-throttled.

After each successful reconstruction, the estimator broadcasts the dynamic TF
`camera_color_optical_frame -> hrm_base` (using the actual CameraInfo frame ID).
Its translation is the running-mean Base point during the configured initial
window and then the fixed average `^C p_B`; its rotation is `^C R_B`. Internal
fitting still uses the explicitly transformed Base-frame NumPy points. This
lets RViz use the camera optical frame as its Fixed Frame while displaying the
Base-frame PointCloud2 and MarkerArray.

The crop image is queued by the color callback and published by the visualization
worker, independently of depth availability or reconstruction success. Camera inputs and visualization
outputs (images, PointCloud2, MarkerArray) use `BEST_EFFORT`, `VOLATILE`,
`KEEP_LAST(1)` sensor-style QoS. The control-facing `SegmentAngle` output stays
`RELIABLE`, `VOLATILE`, `KEEP_LAST(1)`. Set each RViz Image/Marker display's
Reliability to `Best Effort`; a stale RViz display requesting Reliable is QoS
incompatible and will show no data. Visualization messages are constructed
only while a compatible subscriber exists.
One-time reception logs identify which of the three inputs has arrived during
live-camera diagnosis. The strict RGB/depth timestamp rejection was removed;
processing uses the latest aligned-depth frame available, with the configurable
100 ms stale-depth guard added on 2026-09-13.

## Runtime performance

- Active quartic fits use direct linear least squares. The 3D fit remains
  constrained through `(0, 0, 0)` and no longer runs iterative `curve_fit` for
  a model that is linear in its coefficients.
- D405 `rgb8` data is retained in native channel order. The callback selects
  the ROI from the passthrough NumPy view instead of converting the complete
  848x480 image to BGR first; `postprocess()` selects RGB/BGR OpenCV conversion
  codes explicitly.
- The processing worker sleeps on a new-frame event instead of polling a lock
  every 5 ms. ROS callbacks use `SingleThreadedExecutor`; reconstruction and
  visualization remain separate worker threads.
- Depth neighborhood filtering converts only selected pixel windows to float,
  and dense RViz curve output is capped independently of the 1001 internal fit
  samples.
- OpenCV is limited to one worker for these small images to avoid 24-thread
  fan-out and timing jitter. Configure this with
  `performance.opencv_num_threads` in `config.json`.
- Timing and internal input/output-rate logs are disabled by default. Enable
  `performance.debug_timing_enabled` for RBSC/visualization timing and
  `performance.debug_rate_enabled` for internal Hz logs; their intervals are
  controlled by the adjacent `performance` keys. Typical live core time after
  these changes was 11-20 ms
  (previously about 51-55 ms), with occasional 25-35 ms spikes. D405 metadata
  measured about 29.7 Hz; estimator input/output was generally 24-29 Hz on the
  currently loaded desktop. Multiple Python `ros2 topic hz` image subscribers
  materially reduce this rate and are not a passive benchmark.

The independent C++ package `src/image_rate_probe` now provides low-overhead
`image_rate_probe` and `metadata_rate_probe` executables. A clean 2026-09-11
test stopped the estimator, RViz, and existing `topic hz` subscribers while
leaving the D405 wrapper running. Depth metadata remained 29.93-30.00 Hz with
zero missing nominal periods. With alignment enabled, raw depth, raw color,
and aligned depth Image subscriptions varied roughly from 14 to 26 Hz and had
message timestamp gaps of hundreds of milliseconds; with alignment disabled,
the raw streams returned close to 30 Hz. The stock wrapper applies
`rs2::align` synchronously before publishing both aligned and raw frames, so
CPU alignment was stalling that publication path. The later CUDA librealsense
test documented below brought aligned output back to approximately 30 Hz.

Use the integrated camera/estimator launch so D405 profiles and QoS are applied
through the wrapper's config-file path:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch estimation_pkg d405_estimation_launch.py
```

It selects 848x480x30 color/depth, aligned depth, manual exposure 20000, and
`SENSOR_DATA` camera QoS. The D405 color profile parameter is
`depth_module.color_profile`; `rgb_camera.color_profile` does not select this
device's shared depth-module color stream.

`estimation_pkg/launch/_launch.py` no longer starts the full RQT perspective,
because its additional plugins and queues caused substantial live-image delay.
Run `rqt_image_view` or RViz2 separately when visualization is needed.

## Cable and motor state

- `SurgicalTool::inverse_kinematics()` already calculates East, West, South,
  and North cable lengths from pan and tilt targets.
- Motor target indices 0-3 are currently assigned those four cable results.
- Confirm real hardware index ordering and motor direction signs before physical
  operation.
- `PositionController::update()` now passes both accumulated final pan and final
  tilt inputs to `get_IK_result()`.
- `robot_control` now defaults `motor_output_enabled=false`. All control paths
  pass actuator publication through this explicit gate, and a valid
  `motor_state` must also have been received before a real command is sent.
- `kinematics/move_tool_angle` remains usable in dry-run without a motor
  connection. Each request publishes `kinematics/target_wire_length` as a
  four-element `Float64MultiArray` ordered `[East, West, South, North]` in mm
  and logs the same values. The service inputs are degrees (the former `.srv`
  comments incorrectly said radians).
- The dry-run was exercised on 2026-09-13 with motor output disabled. The IK
  results in `[East, West, South, North]` mm were:
  - `tilt=+10, pan=0`: `[+0.00465, +0.00465, -0.63005, +0.63704]`
  - `tilt=-10, pan=0`: South/North signs reverse.
  - `tilt=0, pan=+10`: `[-0.63005, +0.63704, +0.00465, +0.00465]`
  - `tilt=-5, pan=+5`: `[-0.31474, +0.31881, +0.31881, -0.31474]`
  The preview topic was also received successfully, while every
  `motor_command` was blocked by the dry-run gate.
- Keep dry-run active for software checks. Real output requires an explicit
  `motor_output_enabled:=true` launch/parameter setting and supervised hardware
  readiness; this gate does not replace mechanical limits or emergency stop.

## Friction-model decision

- `damping_friction_model.cpp` and its legacy implementation are not used and
  are deleted in the current worktree.
- They were removed from the `robot_control` CMake target and remaining runtime
  references were removed.
- Keep the generalized dynamics equation's `tau_friction` input.
- Current behavior deliberately sets `tau_friction_ = 0.0`, so the friction
  term has no numerical effect.
- Damping remains `dynamics_params::DAMPING` and is separate from friction.

## Future external-force training dataset

- Defer runtime use of `external_force_estimation.py` until the training and
  model-comparison pipeline has been developed.
- Record the original timestamped ROS streams, preferably in rosbag: color,
  aligned depth, CameraInfo, motor/cable displacement, load-cell signals,
  external-force ground truth, TF/calibration, and the online
  `estimated_segment_angle` result.
- Build the canonical training dataset offline by replaying each complete
  sequence through the same causal `estimation_pkg` postprocess and filter
  configuration used online. Do not process isolated images because the Base
  initialization and temporal filter states depend on sequence history.
- Associate a reconstructed angle with the source image timestamp in its
  header, not with the later wall-clock time at which processing/publishing
  finishes.
- At each image timestamp, interpolate or select the temporally nearest motor,
  load-cell, and external-force samples within a defined tolerance. Reject
  samples outside that tolerance and calibrate fixed sensor latencies before
  training.
- Proposed feature/target mapping: motor or cable displacement + load-cell
  measurements + per-joint pan/tilt angles (and, for a temporal model, their
  causal history) as inputs; synchronized external-force sensor values as the
  target.
- Keep the online angle topic in the recording as a diagnostic so offline and
  live feature extraction can be compared for deployment consistency.

## Legacy backup

The single-angle, single-plane implementation is preserved under:

```text
src/robot_control_pkg/legacy/segment_angle_single_axis/
```

It contains the old topic/subscriber declarations, usage mapping, and the
single-theta planar transformation functions. The `.inc` files are reference
snippets and are intentionally excluded from the build.

## Agreed next development roadmap

Resume the project in the following order. Do not start by enabling closed-loop
or admittance motion on hardware.

1. **Pre-motion safety and convention check**
   - Confirm motor indices and directions for East/West pan and South/North
     tilt, cable-length units, encoder zero/reference positions, pretension,
     joint/cable limits, command saturation, timeout behavior, and the physical
     emergency-stop procedure.
   - Validate the estimated pan/tilt signs and the FK axes against several known
     static poses. Tune the angle filter or invalid-frame hold/reset behavior
     before the estimate is used as feedback.
2. **Open-loop cable IK and motor test**
   - Exercise `SurgicalTool::inverse_kinematics()` with small, single-axis tilt
     commands first (South/North and q1), then pan (East/West and q2), then
     combined tilt/pan.
   - Initially inspect/log the four target cable lengths without motor output;
     then run low-speed, limited-amplitude hardware motion under supervision.
   - Compare commanded motion with load-cell/cable feedback, camera-estimated
     joint angles, and FK/TF. Resolve motor-order, sign, offset, slack, and
     workspace-limit errors before proceeding.
3. **Position feedback control**
   - Use the filtered `estimated_segment_angle` and 3D FK to obtain the current
     tip position. Keep X as the calculated axial coordinate only; control Y
     with q1 tilt and Z with q2 pan in the straight configuration.
   - Verify feedback signs before enabling PID, then tune one axis at a time at
     low gain. Add output/integrator limits, stale-estimate handling, and a safe
     fallback when reconstruction is invalid or delayed. Finally test coupled
     pan/tilt motion.
4. **Motor-era data collection and GUI revision**
   - Modify `record_pkg` and `gui_pkg` only once real motor motion and the final
     command/feedback interfaces can be observed. `gui_py_pkg` now reads the
     four-motor hardware count, labels the rows East/West/South/North, and shows
     the dry-run `kinematics/target_wire_length` values in mm. Its kinematics
     button can request a preview without a connected `motor_state`; manual
     motor operation still requires feedback. Revisit the real command path,
     bounds, and enable-state indication after supervised hardware motion.
   - A later GUI feature may start and stop package/launch groups. Implement it
     through supervised ROS launch-process management with visible process
     state and clean shutdown, not by embedding blocking shell commands in GUI
     callbacks.
   - Record raw timestamped color, aligned depth, CameraInfo, motor/cable
     displacement, load cells, external-force ground truth, TF/calibration,
     commands/control mode, and online segment-angle output. Construct the
     canonical synchronized training table offline as specified in the
     external-force dataset section above.
5. **External-force model training and runtime integration**
   - Train and compare models in the `LSTM-force-estimation` research
     workspace. Keep deployment inference in this repository's
     `estimation_pkg`.
   - Validate inference units, coordinate frame, signs, timestamp/latency,
     uncertainty or out-of-distribution behavior, and live rate before the
     estimate can drive a controller.
6. **Admittance control revision and validation**
   - Feed the validated estimated external force into the admittance model;
     confirm desired/environment-force convention and transform all force and
     displacement quantities into one declared frame.
   - Revisit the active pan/tilt dimensions of the M/B/K matrices, sampling
     time, force filtering, saturation, workspace limits, and reset behavior.
     Test offline/replay and zero-force behavior first, then restrained
     low-gain hardware contact tests. Friction remains intentionally zero in
     the generalized dynamics equation unless a later experiment justifies a
     model.

After the above, perform an integrated launcher test, check timing/QoS under
simultaneous camera, estimation, control, recording, and GUI load, and update
the public README with the stable operator-facing launch/calibration/safety
procedure. `CODEX_HANDOFF.md` remains the detailed development record until
those interfaces are stable.

## Deferred work

1. Validate projected DH pan/tilt signs, angle limits, residual thresholds, and
   temporal noise using the real camera and known robot poses before control.
2. Inspect the new FK TF frames in RViz and verify that XYZ axes match
   X=axial, q1 tilt rotating about Base +Z toward +Y, and q2 pan on the next
   orthogonal axis of the real mechanism.
3. Compare model-derived `hrm_fk_*` frames with camera-reconstructed geometry
   (`estimated_segment_center_*`) before enabling position feedback on physical
   hardware.
4. Update `PositionControl.msg`, `record_pkg`, and GUI only after the robot
   control interface stabilizes.
5. Validate motor ordering, direction, tension-array size, and physical safety
   before hardware tests.

## Verification

Last successful command:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select custom_interfaces robot_control_pkg --symlink-install
```

Both packages built successfully. Compiler warnings remained, including PID
initializer ordering and several existing unused
values/parameters. They were not treated as build failures.

After relocating the runtime estimator, colcon discovered only this package:

```text
estimation_pkg  src/estimation_pkg  (ros.ament_python)
```

The following build also completed successfully:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select custom_interfaces estimation_pkg --symlink-install
```

After implementing the RGB-D 3D reconstruction stage, Python compilation,
synthetic pinhole deprojection, synthetic 3D quartic reconstruction, and the
following package build completed successfully on 2026-09-06:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select estimation_pkg --symlink-install
```

After adding optional Base-origin averaging, spatial depth median filtering,
and per-joint `[q, q_dot]` Kalman filtering, focused synthetic tests and the
following build completed successfully on 2026-09-11:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select custom_interfaces estimation_pkg --symlink-install
```

After the runtime performance work, `py_compile`, focused filter/message/marker
tests, and the following build succeeded on 2026-09-11:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select estimation_pkg --symlink-install
```

After adding the extra hardware segment, the synchronized definition is:

```text
19 geometric segments = os + 17*l + le
20 boundary points = P0 ... P19
18 revolute joints = q1 ... q18 = 9 pan/tilt pairs
os = l = le = 4.33 mm, total length = 82.27 mm
```

The following build completed successfully on 2026-09-11:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  custom_interfaces estimation_pkg robot_control_pkg
```

Three focused estimation tests verify the `(20,3)` boundary, `(19,3)` center
and direction, and 18-angle shapes; exclusion of the fixed `os` direction from
joint inversion; and correction of a 4% measured-length error to the fixed
82.27 mm hardware length. All passed. The two C++ FK gtests verify the 82.27 mm
straight tip and that q1 acts after `os`; both passed. Package-wide legacy lint
still reports pre-existing formatting failures in both packages and an online
XML schema lookup failure, but these do not affect the focused numerical tests
or the successful build.

The robot-control runtime loops were subsequently corrected on 2026-09-11:
the inactive dynamics thread now sleeps at 30 Hz instead of busy-spinning one
CPU core, and the position/admittance thread sleeps exactly once per iteration
instead of twice. Segment-angle sharing between ROS callbacks and control
threads is mutex-protected, `control_mode_` is atomic, the initial
`control_mode` parameter is applied, and reliable QoS now correctly uses a
validated depth of one. The package rebuilt successfully. Live execution was
not started automatically because the node contains physical motor-command
paths; validate its CPU and topic rates during the next supervised hardware
run.

The standalone C++ image/metadata rate probe also built successfully:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select image_rate_probe
```

### CUDA RealSense alignment

On 2026-09-11, librealsense `v2.55.1` was built with
`BUILD_WITH_CUDA=ON` for the RTX 3060 (`sm_86`), and realsense-ros `4.55.1`
was rebuilt against it in the isolated, git-ignored `.cuda_realsense/`
directory. The apt/system RealSense installation was not overwritten.

Runtime verification confirmed that the camera process mapped
`.cuda_realsense/install/librealsense/lib/librealsense2.so.2.55.1`, loaded
`libcuda.so`, and appeared as a GPU compute process using about 114 MiB.
With the camera at 848x480x30 and alignment enabled, the low-overhead C++
probe measured aligned depth at 29.60, 29.59, and 29.79 Hz over consecutive
five-second windows. The earlier CPU-alignment measurements were roughly
14-15 Hz.

Use the checked-in helpers to reproduce or run it:

```bash
./scripts/build_realsense_cuda.sh
./scripts/run_realsense_cuda.sh
```

The `launcher` package now contains `cuda_realsense.launch.py` and
`cuda_estimation.launch.py`. These locate the workspace-local CUDA build,
prepend its realsense-ros prefix and libraries to the child environment, and
include the D405 configuration without spawning a nested `ros2 launch` shell.
The primary `system.launch.py` also includes `cuda_realsense.launch.py` in
place of its former apt/CPU camera include. Legacy demo launch files retain
their previous camera settings.
After sourcing the normal workspace, use either:

```bash
ros2 launch launcher cuda_realsense.launch.py
ros2 launch launcher cuda_estimation.launch.py
```

The run helper is retained as a convenience wrapper around the first command.
The CUDA launch environment is required; otherwise the loader may select the
apt CPU library from `/opt/ros/humble`.

The integrated launch also verified the configured 848x480x30 profile and
BEST_EFFORT camera endpoints. Repeated live restarts later exposed an external
V4L2/USB disconnect (`VIDIOC_QBUF: No such device`); replug/recover the D405
before the next camera run rather than treating this driver state as an
estimator exception.

## Worktree safety

- The worktree contains uncommitted changes from both the user and Codex.
- Do not reset, restore, overwrite, or delete unrelated changes.
- Inspect `git status` and `git diff` before editing overlapping files.
- Do not commit or push unless the user explicitly requests it.
- Never put credentials, tokens, passwords, or private machine configuration in
  this file.

## Maintenance policy

Codex should update this file without waiting for a separate user request when
one of the following occurs:

- a control, coordinate-frame, topic, message, or hardware-mapping decision is
  made;
- a material implementation milestone is completed;
- build/test status changes;
- a blocker, safety concern, or important deferred task appears;
- work is being prepared for handoff to another computer or session.

Do not update it for trivial formatting, comments, or exploratory commands.
Keep it concise, factual, and consistent with the repository. Mention material
handoff updates briefly in the final response.
