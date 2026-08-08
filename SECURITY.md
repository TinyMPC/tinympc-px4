# Security and safety reporting

TinyMPC-PX4 is an experimental, non-flight-certified research prototype.

Report software vulnerabilities privately through GitHub's security-advisory
workflow when available. Do not include credentials, private flight logs,
vehicle identifiers, or sensitive hardware details in a public issue.

Control-performance problems, solver failures, estimator discontinuities, and
unsafe mode transitions are also important, but they are not automatically
software-security vulnerabilities. For those reports, open an issue containing
the smallest reproducible SITL or native test case and sanitized diagnostics.

No checked-in configuration is qualified for operation on physical PX4
hardware. See the hardware gates in the README and
`docs/full_state_actuator_constraints.md` before adapting a module for flight.
