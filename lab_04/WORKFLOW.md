# Lab 04 workflow checklist

Repo root is `~/dev/cpe4700` (above `dev`-style paths, so use `git mv lab_02 modules`).

## 1. Rename on main
```
git switch main && git pull
git mv lab_02 modules
# build files: none reference lab_02 yet; check with:  grep -rn lab_02 .
git add -A && git commit -m "Rename shared module directory"
```
## 2. Shared modules on main
`speed_control.c/.h` are already in `modules/`; set the deadband/gain TODOs,
(optionally fix encoder.c sign/stale-speed), then:
```
git add modules/ && git commit -m "Add speed_control module"
git switch -c lab_04
```
(The untracked `lab_04/` dir carries over when you switch; PDF order is modules first.)
## 3. Build and run (sudo is likely needed for sysfs PWM / GPIO)
```
cd lab_04 && make
sudo ./lab04 deadband     # note positive_deadband / negative_deadband
sudo ./lab04 step         # -> step_response.csv
sudo ./lab04 speed        # -> speed_control.csv
python3 analyze.py
```
Add `*.csv` and `lab04` to `.gitignore` (CSVs are not submitted).
## 4. Commit and push
```
git add main.c Makefile analyze.py && git commit -m "Add Lab 04 test files"
git push -u origin lab_04     # then merge into main when finished
```
