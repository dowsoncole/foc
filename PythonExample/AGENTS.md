# Repository instructions for coding agents

## Mandatory change log

Every meaningful code, test, configuration, UI, protocol, or documentation change in this directory must be recorded in `doc/change_log.md` in the same task.

Append a dated entry containing:

1. the user request or problem being addressed;
2. the implementation and important design decisions;
3. every materially changed file and its purpose;
4. validation commands and results;
5. known limitations, hardware checks not performed, and suggested next steps.

Do not overwrite earlier entries. Keep the newest entry at the top of the history section so a future GPT session can recover context quickly. If a change is reverted, add a new entry explaining the revert instead of deleting its original record.

## Issue tracking

When work corresponds to `doc/issue.md`, update its checkbox and verification notes only after the implementation has been tested.
