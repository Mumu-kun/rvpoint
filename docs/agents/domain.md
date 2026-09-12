# Domain Docs

How the engineering skills consume this repository's domain documentation when exploring the codebase.

## Before exploring, read these

- **`docs/CONTEXT.md`**: The authoritative domain model and glossary. Always read this for canonical naming before designing or proposing changes.
- **`docs/adr/`**: Read ADRs that touch the subsystem you are about to modify (e.g. ADR-0010 for zero-heap, ADR-0011 for zero-vtable functors, ADR-0012 for pipeline manager, ADR-0013 for multi-core and slot lifetime).

## File structure

Single-context repository conforming to the inviolate 7-folder root policy:

```
rvpoint/
├── docs/
│   ├── CONTEXT.md           ← Domain glossary & terminology
│   ├── adr/                 ← System-wide architecture decision records
│   ├── plans/               ← Specifications & local work tickets
│   └── agents/              ← Agent skill configuration
├── src/                     ← Pure static library (librvpoint.a)
├── eval/                    ← Standalone pipelines, benchmarks, tests
└── ...
```

## Use the glossary's vocabulary

When naming a domain concept (in code, commit messages, issue titles, specs, tests), strictly use the term defined in `docs/CONTEXT.md`. Do not drift to synonyms the glossary explicitly avoids.

## Flag ADR conflicts

If proposed changes contradict an existing ADR, surface it explicitly rather than silently overriding it.

