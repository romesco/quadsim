"""Nox sessions."""
from pathlib import Path
from textwrap import dedent

import nox
from nox.sessions import Session

# import nox_poetry.patch  # noqa

project = "quadsim"
package = "quadsim"
package_path = f"{project}/{package}"
tests_path = f"{project}/tests"

# NOTE: Dev Requirements are defined in setup.py
# however we explicitly list just those needed for tests here as our
# workflow only install reqs needed for a specific session, not all dev reqs.
test_reqs = ["coverage", "pytest", "pygments"]

python_versions = ["3.7"]

nox.options.sessions = (
    "pre-commit",
    "tests",
)


def activate_virtualenv_in_precommit_hooks(session: Session) -> None:
    """Activate virtualenv in hooks installed by pre-commit.

    This function patches git hooks installed by pre-commit to activate the
    session's virtual environment. This allows pre-commit to locate hooks in
    that environment when invoked from git.

    Args:
        session: The Session object.
    """
    if session.bin is None:
        return

    virtualenv = session.env.get("VIRTUAL_ENV")
    if virtualenv is None:
        return

    hookdir = Path(".git") / "hooks"
    if not hookdir.is_dir():
        return

    for hook in hookdir.iterdir():
        if hook.name.endswith(".sample") or not hook.is_file():
            continue

        text = hook.read_text()
        bindir = repr(session.bin)[1:-1]  # strip quotes
        if not (
            Path("A") == Path("a") and bindir.lower() in text.lower() or bindir in text
        ):
            continue

        lines = text.splitlines()
        if not (lines[0].startswith("#!") and "python" in lines[0].lower()):
            continue

        header = dedent(
            f"""\
            import os
            os.environ["VIRTUAL_ENV"] = {virtualenv!r}
            os.environ["PATH"] = os.pathsep.join((
                {session.bin!r},
                os.environ.get("PATH", ""),
            ))
            """
        )

        lines.insert(1, header)
        hook.write_text("\n".join(lines))


@nox.session(name="pre-commit", python=python_versions)
def precommit(session: Session) -> None:
    """Lint using pre-commit."""
    args = session.posargs or ["run", "--all-files", "--show-diff-on-failure"]
    session.install(
        # when adding a new hook, ensure:
        # 1. add dep to dev_requirements in setup.py
        # 2. the package name is added here (so that nox installs it to virtualenv)
        # 3. the hook is added to pre-commit-config.yaml
        "black",
        "flake8",
        "flake8-bugbear",
        "pep8-naming",
        "pre-commit",
        "pre-commit-hooks",
        "reorder-python-imports",
    )
    session.run("pre-commit", *args)
    if args and args[0] == "install":
        activate_virtualenv_in_precommit_hooks(session)


@nox.session(python=python_versions)
def tests(session: Session) -> None:
    """Run the test suite."""
    session.install(".")
    session.install(*test_reqs)
    # session.run("poetry", "install", external=True)
    session.run("pip", "install", ".[dev]")

    try:
        session.run("coverage", "run", "-m", "pytest", *session.posargs)
    finally:
        session.notify("coverage_report")


@nox.session
def coverage_report(session: Session) -> None:
    """Generate coverage report from tests output."""
    session.install("coverage[toml]")

    if any(Path().glob(".coverage.*")):
        session.run("coverage", "combine")

    session.run("coverage", "report")  # show coverage report in CLI
    session.run("coverage", "xml")  # save coverage report to xml for upload to codecov
