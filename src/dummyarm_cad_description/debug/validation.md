# Validation Report: Dummy_URDF

## Status: PASS

## Summary

| Metric | Value |
|--------|-------|
| Links | 7 |
| Joints | 6 |
| Assemblies | 17 |
| Root | `base_link` |
| Errors | 0 |
| Warnings | 0 |

## Kinematic Tree

```
base_link [BAKE] [cached_existing]
  └─ _1 [revolute]
    J1 [BAKE] [cached_existing]
      └─ _2 [revolute]
        J2J3 [BAKE] [cached_existing]
          └─ _3 [revolute]
            J4 [BAKE] [cached_existing]
              └─ _4 [revolute]
                J5 [BAKE] [cached_existing]
                  └─ _5 [revolute]
                    J6 [BAKE] [cached_existing]
                      └─ _6 [continuous]
                        tool0 [BAKE] [cached_existing]
```

## Collision Geometry

| Link | Source | Shape/File |
|------|--------|------------|
| `J1` | cached_existing | — |
| `J2J3` | cached_existing | — |
| `J4` | cached_existing | — |
| `J5` | cached_existing | — |
| `J6` | cached_existing | — |
| `base_link` | cached_existing | — |
| `tool0` | cached_existing | — |

## Mesh Bake Offsets

Links where joint frame ≠ component origin. Visual/inertial/collision origins shifted.

| Link | Offset (mm) |
|------|-------------|
| `J1` | (15.0, -35.0, 39.0) |
| `J2J3` | (0.0, 0.0, 32.0) |
| `J4` | (37.8, -141.0, -32.0) |
| `J5` | (-39.9, 0.0, 49.5) |
| `J6` | (-8.7, 2.3, -17.1) |
| `base_link` | (0.0, 0.0, 0.0) |
| `tool0` | (0.0, 0.0, 6.0) |
