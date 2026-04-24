# dosiz project conventions

## Workflow

- When you reach a summary/end-of-turn for a unit of work, commit and push
  to origin/main without asking first. Don't wait for explicit "commit" or
  "push" instructions.
- Submodule `dosbox-staging` always shows dirty ("modified content") because
  the Makefile patches sdlmain.cpp at build time from
  `patches/sdlmain-expose-setup.patch`. That's expected -- don't commit the
  submodule state or try to clean it.
