# AGENTS.md

Project-wide agent instructions, shared by every tool that reads `AGENTS.md` (OpenAI Codex CLI, Codex IDE extensions, and any other AGENTS.md-aware coding agent). Claude Code does **not** read this file directly — it imports the same content via `@AGENTS.md` from [CLAUDE.md](CLAUDE.md).

If you edit this file, edit [CLAUDE.md](CLAUDE.md)'s `## Behavioral guidelines (general)` section too, or just re-edit both — they share a single source of intent even if their text diverges slightly for tool-specific concerns.

## Project pointer — `bowen_ADAS`

This repo is a **three-machine HIL ADAS stack** with live hardware and a SIL fallback. Read [CLAUDE.md](CLAUDE.md) for the full repository overview, build/test commands, cross-codebase contracts, and the safety invariants. Below are the cross-tool conventions that complement whatever is in `CLAUDE.md`.

Hard rule of thumb: when a project-specific instruction in [CLAUDE.md](CLAUDE.md) (e.g. "never `SIGKILL` the bridge") and a general guideline here disagree, **the project-specific one wins** — see [CLAUDE.md](CLAUDE.md)'s safety invariants and the note in its `## Behavioral guidelines` section.

## Behavioral guidelines (general) — load-bearing, do not strip

> **Status: deliberate, not decoration.** This file and the `@AGENTS.md` import at the bottom of [CLAUDE.md](CLAUDE.md) are the wiring that makes **Codex CLI and Claude Code read the same four principles**. The embedded content below is what every AGENTS.md-aware tool sees natively; Claude Code gets it via the `@AGENTS.md` line. **Do not remove this section** under "Simplicity First" — it is the duplication-killer between Claude Code and Codex, not excess mechanism.

The four principles below are vendored from [forrestchang/andrej-karpathy-skills](https://github.com/forrestchang/andrej-karpathy-skills) at [`tools/karpathy-guidelines/`](tools/karpathy-guidelines/) — update with `cd tools/karpathy-guidelines && git pull`.

> **Tradeoff:** these guidelines bias toward caution over speed. For trivial tasks (typo fix, one-line rename), use judgment.

### 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:

- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them — don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

### 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

### 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:

- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, **mention it — don't delete it**.

When your changes create orphans:

- Remove imports / variables / functions that **your** changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: every changed line should trace directly to the user's request.

### 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:

- "Add validation" → "Write tests for invalid inputs, then make them pass."
- "Fix the bug" → "Write a test that reproduces it, then make it pass."
- "Refactor X" → "Ensure tests pass before and after."

For multi-step tasks, state a brief plan before executing:

```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

---

**These guidelines are working if:** fewer unnecessary changes in diffs, fewer rewrites due to over-complication, and clarifying questions come before implementation rather than after mistakes.
