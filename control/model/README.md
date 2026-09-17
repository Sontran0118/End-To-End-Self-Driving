# jetson_port/model — mirror of the live model port

These five files are a **mirror**, not the source of truth. The files that actually
run live in `/home/tran/openpilot_jetson/`:

| file | live source |
|---|---|
| `op_frame.py`    | `/home/tran/openpilot_jetson/op_frame.py` |
| `op_stream.py`   | `/home/tran/openpilot_jetson/op_stream.py` |
| `op_parse.py`    | `/home/tran/openpilot_jetson/op_parse.py` |
| `op_run.py`      | `/home/tran/openpilot_jetson/op_run.py` |
| `op_pipeline.py` | `/home/tran/openpilot_jetson/op_pipeline.py` |

Nothing imports from this directory. Every consumer in `../control_stack/` starts with
`sys.path.insert(0, "/home/tran/openpilot_jetson")`, so the live copies always win —
this directory exists so the fork carries the model port with it, not so it executes.

## Keeping it in sync

```sh
for f in op_frame.py op_stream.py op_parse.py op_run.py op_pipeline.py; do
  cp /home/tran/openpilot_jetson/$f jetson_port/model/$f
done
```

## Why this warning exists

The mirror silently rotted. Before it was last synced it had drifted badly enough to be
actively misleading:

- `op_frame.py` was 83 lines against the live 313, and its docstring claimed the
  pipeline "matches openpilot exactly" while it had the Y-quadrant channel order
  transposed, studio-swing YUV instead of BT.601 full range, `big_img` as a byte copy
  of `img`, no ring buffer for the 200 ms frame spacing, and `INTER_LINEAR` where
  openpilot rounds to nearest.
- `op_parse.py` referenced `MC.POSITION`, which does not exist — `AttributeError` on
  first call. It is `Plan.POSITION`.
- `op_run.py` was missing the `sys.path` insert it needs to import `constants` at all.

So the copies here were not merely old, they were broken, while reading as authoritative.
If you change a model file, sync it here in the same commit or this happens again.

## Portability caveat

These files are not self-contained on a fresh clone regardless of syncing — they carry
absolute `/home/tran/...` paths for the TRT engine, the ONNX model, `modeld/constants.py`
and `curvature_lib.py`. Treat this directory as a record of what ran on this box, not as
a drop-in module for another machine.
