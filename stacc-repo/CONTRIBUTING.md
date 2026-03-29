# Contributing to stacc

Thank you for your interest in contributing to stacc! This document provides guidelines for contributing.

## Getting started

1. Fork the repository
2. Clone your fork: `git clone https://github.com/YOUR_USERNAME/stacc.git`
3. Create a branch: `git checkout -b feature/your-feature`
4. Install dev dependencies: `pip install -e ".[dev]"`
5. Make your changes
6. Run tests: `pytest`
7. Push and open a PR

## Development setup

```bash
git clone https://github.com/YOUR_ORG/stacc.git
cd stacc
python -m venv venv
source venv/bin/activate  # or venv\Scripts\activate on Windows
pip install -e ".[dev]"
```

## Project structure

```
stacc/
├── stacc/          # Core Python package
│   ├── cli.py      # CLI commands
│   ├── core/       # Robot model fundamentals
│   ├── kinematics/ # Math engine
│   ├── hal/        # Hardware abstraction
│   ├── sensors/    # Sensor drivers
│   ├── sim/        # Simulation engine
│   ├── ai/         # AI/ML toolkit
│   ├── robot/      # Robot implementations
│   ├── control/    # Controllers
│   ├── gait/       # Locomotion
│   └── gripper/    # End effectors
├── gui/            # GUI platform
├── tests/          # Tests
└── docs/           # Documentation
```

## Code style

- Python: follow PEP 8, use type hints
- C++: follow Google C++ style guide
- Max line length: 100 characters
- Use docstrings for all public functions/classes
- Write tests for new features

## Commit messages

Use conventional commits:

```
feat: add inverse kinematics solver
fix: correct quaternion normalization in transforms
docs: update hardware support table
test: add PID controller unit tests
refactor: simplify URDF parser
```

## Pull request process

1. Update documentation if needed
2. Add tests for new features
3. Ensure all tests pass
4. Update CHANGELOG.md
5. Request review from a maintainer

## Priority areas

We especially welcome contributions in these areas:

- **Kinematics engine** — FK, IK, dynamics solvers
- **Simulation backends** — MuJoCo, Bullet, DART wrappers
- **Hardware drivers** — new board/sensor/motor support
- **Robot templates** — new robot types and examples
- **Documentation** — tutorials, API docs, examples
- **Tests** — unit tests, integration tests

## Reporting bugs

Use GitHub Issues with the bug report template. Include:

- stacc version (`stacc --version`)
- Python version
- OS and hardware
- Steps to reproduce
- Expected vs actual behavior

## Feature requests

Use GitHub Issues with the feature request template. Describe:

- The problem you're trying to solve
- Your proposed solution
- Alternatives you've considered

## Code of conduct

Be respectful, inclusive, and constructive. We're building something for the entire robotics community.

## License

By contributing, you agree that your contributions will be licensed under the MIT License.
