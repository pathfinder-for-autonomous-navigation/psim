Before opening the PR, think: 
- Does this PR break psim, can `main.m` still run, and do all tests in `run_tests.m` pass?
- Is this PR bulletproof? Can I think of no ways to improve this PR beyond its current state?
- Is the PR short enough to ensure a quick but thorough review, but long enough to represent a complete feature? For large state machines, let's try opening PRs for only a few states at a time.

If the answer to any of these questions is "no", refactor your code and open your PR when the answer to both questions is "yes".

--------------------------

# PR Title

Fixes #. (Optional; sometimes PRs don't have an associated ticket)

### Summary of changes
- 
- 
- 

### Ptest Effects
What effect will this have on the ptest interface? Don't use uint64 because uint64 is not python compatible. Be careful when changing sensor readings or actuator commands because these are directly sent to and recieved from the flight computer.

### Testing
Such as your unit and functional testing. The testing should be at a level appropriate to demonstrate that your feature is sufficiently tested.
For new MATLAB functions, added a simple test script to `MATLAB/test` directory and make sure it is called in `MATLAB/run_tests.m`. Plotting functions and functions with IO or other effects should not be tested in `MATLAB/run_tests.m`. Unit tests should be fast, deterministic and silent if they pass, and give lots of debug info only when they fail.

If the functionality can only be tested with a long sim, give instructions for how to do those sims and get the plots. If needed, add a new plotting function to `MATLAB/plot` to help others reproduce your results.

If your update does not require significant testing, argue why, or if you're deferring testing to another PR, create an issue ticket for the deferral and link it.

### Constants
If your PR creates any constants, list them here and indicate whether or not you've added them to the recurring constants review.
If you change or add any fields in `const`, `main_state`, sensor readings or actuator commands make sure to document that change in `MATLAB/README.md`.

| Constant                 | Location             | Added to review?   |
|--------------------------|----------------------|--------------------|
|                          |                      | :white_check_mark: |
|                          |                      | :x: : Why not?     |


### Documentation Evidence
Post to ReadTheDocs, or
- Argue that your inline documentation is sufficient.
- Create a issue ticket deferring the documentation task.
