# quadsim

Quadruped Simulation &amp; Control Framework

## dev

```bash
# clone the repo
git clone git@github.com:romesco/quadsim.git

# activate your env (sub w/ your fave virtual env manager)
pyenv activate <env-name>

# install deps
pip install -e .[dev]

# configure pre-commit
pre-commit install

# run lint & tests locally
nox
```

## regenerate hydra configs
```bash
# edit the following & add new class
vim configen/conf/quadsim.yaml

# rerun configen from root of repo
configen --config-dir configen/conf --config-name quadsim
```

## run basic hydra example
```bash
python examples/run_hydra.py
```



