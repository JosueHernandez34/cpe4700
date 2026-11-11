"""Read the numbers in robot_params.h so Python scripts use the same values as the C code.

    import sys; sys.path.insert(0, "../modules")
    import robot_params as rp
    rp.ROBOT_KP, rp.ROBOT_MAX_DUTY, ...

Only lines of the form "#define ROBOT_NAME <number or arithmetic using earlier ROBOT_ names>" are read.
"""
import os
import re

_HEADER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_params.h")


def _load():
    values = {}
    pat = re.compile(r"^#define\s+(ROBOT_[A-Z0-9_]+)\s+(.+?)\s*(?:/\*.*)?$")
    with open(_HEADER) as f:
        for line in f:
            m = pat.match(line.strip())
            if not m:
                continue
            name, expr = m.groups()
            expr = re.sub(r"/\*.*?\*/", "", expr).strip()
            try:
                values[name] = eval(expr, {"__builtins__": {}}, dict(values))
            except Exception:
                int_form = expr.rstrip("uUlL")
                try:
                    values[name] = eval(int_form, {"__builtins__": {}}, dict(values))
                except Exception:
                    pass
    return values


PARAMS = _load()
globals().update(PARAMS)

if __name__ == "__main__":
    for k, v in PARAMS.items():
        print(f"{k} = {v}")
