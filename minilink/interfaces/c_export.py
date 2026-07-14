import ctypes
import math
import platform
import subprocess
import tempfile
from pathlib import Path

import jax
import numpy as np

from minilink.core.system import System


def get_shape(v):
    if type(v).__name__ == "Literal":
        val = np.asarray(v.val)
        return val.shape
    if hasattr(v, "aval") and hasattr(v.aval, "shape"):
        return v.aval.shape
    return ()


def get_size(v):
    return math.prod(get_shape(v))


def clean_name(v):
    if type(v).__name__ == "Literal":
        return None
    import re

    return re.sub(r"[^a-zA-Z0-9]", "_", str(v))


def transpile_jaxpr_to_c(closed_jaxpr, func_name="evaluate"):
    jaxpr = getattr(closed_jaxpr, "jaxpr", closed_jaxpr)

    def get_val(v, i):
        if type(v).__name__ == "Literal":
            val = np.asarray(v.val).flatten()[i]
            if np.isinf(val):
                return "INFINITY" if val > 0 else "-INFINITY"
            if np.isnan(val):
                return "NAN"
            return f"{float(val)}f"

        name = clean_name(v)
        if get_size(v) == 1:
            return name
        return f"{name}[{i}]"

    lines = []
    lines.append("#include <math.h>")
    lines.append("")

    # Generate C function signature
    c_args = []
    for var in jaxpr.invars:
        name = clean_name(var)
        size = get_size(var)
        if size == 1:
            c_args.append(f"const float {name}")
        else:
            c_args.append(f"const float* {name}")

    for var in jaxpr.outvars:
        name = clean_name(var)
        size = get_size(var)
        if size == 1:
            c_args.append(f"float* out_{name}")
        else:
            c_args.append(f"float* out_{name}")

    lines.append(f"void {func_name}({', '.join(c_args)}) {{")

    # Declare intermediate variables
    for eqn in jaxpr.eqns:
        for outvar in eqn.outvars:
            name = clean_name(outvar)
            if name is None:
                continue
            size = get_size(outvar)
            if size == 1:
                lines.append(f"  float {name};")
            else:
                lines.append(f"  float {name}[{size}];")

    # Declare and initialize constvars
    if hasattr(closed_jaxpr, "consts"):
        for var, c in zip(jaxpr.constvars, closed_jaxpr.consts):
            name = clean_name(var)
            size = get_size(var)
            vals = np.asarray(c).flatten()
            if size == 1:
                lines.append(f"  const float {name} = {float(vals[0])}f;")
            else:
                val_strs = [f"{float(v)}f" for v in vals]
                lines.append(
                    f"  const float {name}[{size}] = {{{', '.join(val_strs)}}};"
                )

    lines.append("")

    # Process equations
    for eqn in jaxpr.eqns:
        prim = eqn.primitive.name
        outvar = eqn.outvars[0]
        size = get_size(outvar)

        if prim in ["add", "sub", "mul", "div"]:
            op_map = {"add": "+", "sub": "-", "mul": "*", "div": "/"}
            op = op_map[prim]
            for i in range(size):
                out = get_val(outvar, i)
                in0 = get_val(eqn.invars[0], i)
                in1 = get_val(eqn.invars[1], i)
                lines.append(f"  {out} = {in0} {op} {in1};")

        elif prim in ["gt", "lt", "ge", "le", "eq", "ne", "and", "or"]:
            op_map = {
                "gt": ">",
                "lt": "<",
                "ge": ">=",
                "le": "<=",
                "eq": "==",
                "ne": "!=",
                "and": "&&",
                "or": "||",
            }
            op = op_map[prim]
            for i in range(size):
                out = get_val(outvar, i)
                in0 = get_val(eqn.invars[0], i)
                in1 = get_val(eqn.invars[1], i)
                lines.append(f"  {out} = ({in0} {op} {in1}) ? 1.0f : 0.0f;")

        elif prim == "neg":
            for i in range(size):
                lines.append(f"  {get_val(outvar, i)} = -{get_val(eqn.invars[0], i)};")

        elif prim in ["sin", "cos", "tan", "sqrt", "abs"]:
            func_map = {
                "sin": "sinf",
                "cos": "cosf",
                "tan": "tanf",
                "sqrt": "sqrtf",
                "abs": "fabsf",
            }
            func = func_map[prim]
            for i in range(size):
                lines.append(
                    f"  {get_val(outvar, i)} = {func}({get_val(eqn.invars[0], i)});"
                )

        elif prim == "sign":
            for i in range(size):
                out = get_val(outvar, i)
                in0 = get_val(eqn.invars[0], i)
                lines.append(f"  {out} = ({in0} > 0.0f) - ({in0} < 0.0f);")

        elif prim == "max":
            for i in range(size):
                lines.append(
                    f"  {get_val(outvar, i)} = fmaxf({get_val(eqn.invars[0], i)}, {get_val(eqn.invars[1], i)});"
                )

        elif prim == "min":
            for i in range(size):
                lines.append(
                    f"  {get_val(outvar, i)} = fminf({get_val(eqn.invars[0], i)}, {get_val(eqn.invars[1], i)});"
                )

        elif prim == "select_n":
            # select_n(cond, case0, case1) -> cond ? case1 : case0 (if cond is bool/0-1)
            # JAX select_n takes an integer index. If it's a bool, 0 -> case0, 1 -> case1
            for i in range(size):
                out = get_val(outvar, i)
                cond = get_val(eqn.invars[0], i)
                case0 = get_val(eqn.invars[1], i)
                case1 = get_val(eqn.invars[2], i)
                lines.append(f"  {out} = ({cond} != 0.0f) ? {case1} : {case0};")

        elif prim == "broadcast_in_dim":
            in_shape = get_shape(eqn.invars[0])
            out_shape = get_shape(outvar)
            broadcast_dimensions = eqn.params["broadcast_dimensions"]
            for out_idx in np.ndindex(out_shape if out_shape else (1,)):
                if not out_shape:
                    out_idx = ()
                in_idx = [0] * len(in_shape)
                for i, dim in enumerate(broadcast_dimensions):
                    in_idx[i] = out_idx[dim]
                flat_out = np.ravel_multi_index(out_idx, out_shape) if out_shape else 0
                flat_in = np.ravel_multi_index(in_idx, in_shape) if in_shape else 0
                lines.append(
                    f"  {get_val(outvar, flat_out)} = {get_val(eqn.invars[0], flat_in)};"
                )

        elif prim in ["reshape", "squeeze"]:
            for i in range(size):
                lines.append(f"  {get_val(outvar, i)} = {get_val(eqn.invars[0], i)};")

        elif prim == "concatenate":
            dimension = eqn.params["dimension"]
            dummy_inputs = []
            for i, inv in enumerate(eqn.invars):
                shape = get_shape(inv)
                if shape == ():
                    shape = (1,)
                arr = np.empty(shape, dtype=object)
                for idx in np.ndindex(shape):
                    flat_idx = np.ravel_multi_index(idx, shape)
                    arr[idx] = (i, flat_idx)
                dummy_inputs.append(arr)
            concat_arr = np.concatenate(dummy_inputs, axis=dimension)
            for out_flat, (in_arr_idx, in_flat_idx) in enumerate(concat_arr.flatten()):
                lines.append(
                    f"  {get_val(outvar, out_flat)} = {get_val(eqn.invars[in_arr_idx], in_flat_idx)};"
                )

        elif prim == "slice":
            in_shape = get_shape(eqn.invars[0])
            arr = np.arange(math.prod(in_shape)).reshape(in_shape)
            starts = eqn.params["start_indices"]
            limits = eqn.params["limit_indices"]
            strides = eqn.params.get("strides")
            if strides is None:
                strides = [1] * len(starts)
            slices = tuple(slice(s, l, st) for s, l, st in zip(starts, limits, strides))
            sliced_arr = arr[slices]
            for out_flat, in_flat_idx in enumerate(sliced_arr.flatten()):
                lines.append(
                    f"  {get_val(outvar, out_flat)} = {get_val(eqn.invars[0], in_flat_idx)};"
                )

        elif prim == "dot_general":
            lhs_shape = get_shape(eqn.invars[0])
            rhs_shape = get_shape(eqn.invars[1])
            out_shape = get_shape(outvar)
            (lhs_contracting, rhs_contracting), (lhs_batch, rhs_batch) = eqn.params[
                "dimension_numbers"
            ]

            out_terms = [[] for _ in range(math.prod(out_shape) if out_shape else 1)]

            for lhs_idx in np.ndindex(lhs_shape if lhs_shape else (1,)):
                if not lhs_shape:
                    lhs_idx = ()
                for rhs_idx in np.ndindex(rhs_shape if rhs_shape else (1,)):
                    if not rhs_shape:
                        rhs_idx = ()
                    match = True
                    for c_l, c_r in zip(lhs_contracting, rhs_contracting):
                        if lhs_idx[c_l] != rhs_idx[c_r]:
                            match = False
                    for b_l, b_r in zip(lhs_batch, rhs_batch):
                        if lhs_idx[b_l] != rhs_idx[b_r]:
                            match = False
                    if not match:
                        continue

                    out_idx = []
                    for b_l in lhs_batch:
                        out_idx.append(lhs_idx[b_l])
                    for i in range(len(lhs_shape)):
                        if i not in lhs_contracting and i not in lhs_batch:
                            out_idx.append(lhs_idx[i])
                    for i in range(len(rhs_shape)):
                        if i not in rhs_contracting and i not in rhs_batch:
                            out_idx.append(rhs_idx[i])

                    flat_out = (
                        np.ravel_multi_index(tuple(out_idx), out_shape)
                        if out_shape
                        else 0
                    )
                    flat_lhs = (
                        np.ravel_multi_index(lhs_idx, lhs_shape) if lhs_shape else 0
                    )
                    flat_rhs = (
                        np.ravel_multi_index(rhs_idx, rhs_shape) if rhs_shape else 0
                    )

                    out_terms[flat_out].append(
                        f"{get_val(eqn.invars[0], flat_lhs)} * {get_val(eqn.invars[1], flat_rhs)}"
                    )

            for i in range(math.prod(out_shape) if out_shape else 1):
                out_val = get_val(outvar, i)
                if not out_terms[i]:
                    lines.append(f"  {out_val} = 0.0f;")
                else:
                    lines.append(f"  {out_val} = {' + '.join(out_terms[i])};")

        elif prim == "scatter":
            # For 1D contiguous slices, scatter takes (operand, indices, updates).
            # indices contains the start index.
            operand = eqn.invars[0]
            indices = eqn.invars[1]
            updates = eqn.invars[2]

            # 1. Copy operand to outvar
            for i in range(get_size(operand)):
                lines.append(f"  {get_val(outvar, i)} = {get_val(operand, i)};")

            # 2. Apply updates
            num_updates = get_size(updates)
            idx_var = get_val(indices, 0)
            array_name = get_val(outvar, 0).split("[")[0]
            for i in range(num_updates):
                lines.append(
                    f"  {array_name}[(int){idx_var} + {i}] = {get_val(updates, i)};"
                )
        elif prim == "iota":
            shape = eqn.params["shape"]
            dimension = eqn.params["dimension"]
            size = math.prod(shape)
            iota_arr = np.indices(shape)[dimension]
            for i, val in enumerate(iota_arr.flatten()):
                lines.append(f"  {get_val(outvar, i)} = {val};")
        elif prim in ["convert_element_type", "stop_gradient"]:
            for i in range(size):
                lines.append(f"  {get_val(outvar, i)} = {get_val(eqn.invars[0], i)};")
        else:
            raise NotImplementedError(
                f"JAX primitive '{prim}' is not supported by the C transpiler."
            )

    # Assign to output pointers
    lines.append("")
    for var in jaxpr.outvars:
        name = clean_name(var)
        size = get_size(var)
        if size == 1:
            lines.append(f"  *out_{name} = {name};")
        else:
            for i in range(size):
                lines.append(f"  out_{name}[{i}] = {name}[{i}];")

    lines.append("}")
    return "\n".join(lines)


def export_system_to_c(
    system: System, func_name: str, x_sample=None, u_sample=None, t_sample=0.0
):
    """
    Compile a minilink System and export its equations to a standalone C function.
    """
    evaluator = system.compile(backend="jax")

    # Create dummy inputs based on system dimensions
    if x_sample is None:
        x_sample = np.zeros(system.n)
    if u_sample is None:
        u_sample = np.zeros(system.m)

    x_jax = jax.numpy.array(x_sample, dtype=jax.numpy.float32)
    u_jax = jax.numpy.array(u_sample, dtype=jax.numpy.float32)
    t_jax = jax.numpy.array(t_sample, dtype=jax.numpy.float32)

    # DynamicSystem uses f, StepSystem uses step, Static uses outputs
    if hasattr(evaluator, "f_trace"):

        def func(x, u):
            return evaluator.f_trace(x, u, t_jax)
    elif hasattr(evaluator, "step_trace"):

        def func(x, u):
            return evaluator.step_trace(x, u, 0)
    elif hasattr(evaluator, "outputs_trace"):

        def func(x, u):
            return evaluator.outputs_trace(x, u, t_jax)
    elif hasattr(evaluator, "f"):

        def func(x, u):
            return evaluator.f(x, u, t_jax)
    elif hasattr(evaluator, "step"):

        def func(x, u):
            return evaluator.step(x, u, 0)
    else:

        def func(x, u):
            return evaluator.outputs(x, u, t_jax)

    # Trace tier avoids evaluator-level JIT; disable_jit still needed when
    # subsystem f()/compute() embed jax.jit (e.g. FilteredController).
    with jax.disable_jit():
        closed_jaxpr = jax.make_jaxpr(func)(x_jax, u_jax)

    return transpile_jaxpr_to_c(closed_jaxpr, func_name)


def compile_c_shared(c_source, work_dir=None):
    """Compile C source to a shared library; return the library path.

    Uses ``cc -shared -fPIC``. Raises ``RuntimeError`` if the compiler is
    missing or the build fails.
    """
    work = (
        Path(tempfile.mkdtemp(prefix="minilink_c_export_"))
        if work_dir is None
        else Path(work_dir)
    )
    work.mkdir(parents=True, exist_ok=True)
    src_path = work / "exported.c"
    suffix = ".dylib" if platform.system() == "Darwin" else ".so"
    lib_path = work / f"exported{suffix}"
    src_path.write_text(c_source)

    cmd = ["cc", "-shared", "-fPIC", "-O0", "-o", str(lib_path), str(src_path)]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, check=False)
    except FileNotFoundError as exc:
        raise RuntimeError(
            "C export round-trip requires a C compiler (`cc` not found on PATH)."
        ) from exc
    if result.returncode != 0:
        raise RuntimeError(
            "Failed to compile exported C:\n"
            f"command: {' '.join(cmd)}\n"
            f"stdout:\n{result.stdout}\n"
            f"stderr:\n{result.stderr}"
        )
    return lib_path


def _c_float_arg(arr):
    """Match ``transpile_jaxpr_to_c``: size-1 inputs are passed by value."""
    arr = np.asarray(arr, dtype=np.float32).ravel()
    if arr.size == 1:
        return ctypes.c_float(float(arr[0]))
    return arr.ctypes.data_as(ctypes.POINTER(ctypes.c_float))


def load_exported_c(c_source, func_name, n_out, work_dir=None):
    """Compile generated C and return a Python ``(x, u) -> y`` callable.

    Calls the exported function by argument order (``x``, ``u``, then the
    output buffer). Does not depend on mangled C identifier names.
    """
    lib_path = compile_c_shared(c_source, work_dir=work_dir)
    lib = ctypes.CDLL(str(lib_path))
    c_func = getattr(lib, func_name)
    c_func.restype = None

    def evaluate(x, u):
        x_arr = np.ascontiguousarray(np.asarray(x, dtype=np.float32).ravel())
        u_arr = np.ascontiguousarray(np.asarray(u, dtype=np.float32).ravel())
        out = np.zeros(n_out, dtype=np.float32)
        c_func(
            _c_float_arg(x_arr),
            _c_float_arg(u_arr),
            out.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
        )
        return out

    evaluate.lib_path = lib_path
    return evaluate
