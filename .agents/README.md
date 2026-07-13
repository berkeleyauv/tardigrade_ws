# Agent Context Index

Use these files to orient future Codex/agent sessions before editing the repo.

## Active Files

```text
.agents/architecture.md
```

Current package boundaries, system data flow, safety defaults, and hardware
status. Read this before changing package responsibilities or PX4-facing code.

```text
.agents/current_plan.md
```

Current work plan. This is the active planning file now that electrical
hardware is unavailable and local development is the primary path.

## Legacy Files

```text
.agents/legacy/week_one_plan.md
```

Historical bringup plan from the Jetson/ZED/VectorNav/Pixhawk bench push. Keep
it for context, but do not treat it as the current task list.

## Human-Facing Docs

```text
README.md
SETUP.md
SCRIPTS.md
docs/jetson_zed_px4_startup.md
docs/thruster_mapping.md
```

Keep the README concise. Put short setup and command references in `SETUP.md`
and `SCRIPTS.md`. Put detailed hardware procedures in
`docs/jetson_zed_px4_startup.md`.
