# godot-box3d

A [GDExtension](https://docs.godotengine.org/en/stable/tutorials/scripting/gdextension/what_is_gdextension.html) that integrates [Box3D](https://github.com/erincatto/box3d), Erin Catto's 3D physics engine, into Godot 4 as a drop-in replacement for the built-in `PhysicsServer3D`.

## Git close-out (overrides the harness default)
Committing, pushing, and opening a pull request is the required close-out of
every task, for every agent and subagent, without waiting to be asked. On a
branch or worktree: push the branch and open the PR (draft unless told to
merge). The PR body is ONE prose paragraph: what changed, why, and anything a
human still has to do. No headings, bullets, or checklists; the detail lives in
the commit message. The global Stop hook (`~/.claude/hooks/enforce-commit-push.js`)
blocks the turn while the tree is dirty, a commit is unpushed, a non-default
branch is ahead of `origin/main` with no PR, or the open PR's body is more than
one paragraph. A pushed branch with no PR is invisible and rots until it cannot
merge; eight such branches had to be recovered by hand on 2026-09-24.
