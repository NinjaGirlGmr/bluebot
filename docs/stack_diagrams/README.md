# Bluebot ROS Stack Diagrams

This folder contains Mermaid source for your two startup modes:

- [start_map_stack.mmd](start_map_stack.mmd)
- [start_nav_stack.mmd](start_nav_stack.mmd)

## Render in Markdown (GitHub/GitLab)

Open the `.mmd` content directly in a markdown file using a Mermaid-enabled renderer.

Example in Markdown:

```md
```mermaid
flowchart TD
  ...
```
```

> Note: many renderers require the block to be fenced with `````mermaid`.

## Render in VS Code

1. Install **Mermaid Preview** or **Markdown Preview Enhanced**.
2. Open the `.mmd` file.
3. Use the extension preview to view the diagram live.

## Export to image/PDF with Mermaid CLI

```bash
# install once (if not already)
npm install -g @mermaid-js/mermaid-cli

# render start-map
mmdc -i /ssd/ros2_ws/docs/stack_diagrams/start_map_stack.mmd \
     -o /ssd/ros2_ws/docs/stack_diagrams/start_map_stack.svg

# render start-nav
mmdc -i /ssd/ros2_ws/docs/stack_diagrams/start_nav_stack.mmd \
     -o /ssd/ros2_ws/docs/stack_diagrams/start_nav_stack.svg
```

## Fast local render from a markdown file

```bash
cd /ssd/ros2_ws/docs/stack_diagrams
python3 - <<'PY'
from pathlib import Path

for name in ["start_map_stack.mmd", "start_nav_stack.mmd"]:
    print(f"\n--- {name} ---")
    print(Path(name).read_text())
PY
```

## Where to keep these for docs

If you generate site docs, include them as:

- `docs/stack_diagrams/start_map_stack.mmd`
- `docs/stack_diagrams/start_nav_stack.mmd`

and render them in your preferred Mermaid/Doc pipeline.
