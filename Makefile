# Fly-in - project automation

PYTHON ?= python3
MAP ?= maps/easy/01_linear_path.txt

install:
	$(PYTHON) -m pip install -r requirements.txt

run:
	$(PYTHON) main.py $(MAP) || true

debug:
	$(PYTHON) -m pdb main.py $(MAP)

clean:
	find . -type d -name "__pycache__" -exec rm -rf {} +
	rm -rf .mypy_cache .pytest_cache

lint:
	flake8 .
	mypy . --warn-return-any --warn-unused-ignores --ignore-missing-imports \
		--disallow-untyped-defs --check-untyped-defs

lint-strict:
	flake8 .
	mypy . --strict

.PHONY: install run debug clean lint lint-strict
