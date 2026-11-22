.PHONY: up up-lite up-mock up-software down build check test logs clean
up:            ## Gazebo + GUI + small VLM (needs ~3.5 GB free RAM)
	bash scripts/run.sh full
up-lite:       ## Gazebo + GUI, rule-based planner (no VLM, ~2.5 GB)
	bash scripts/run.sh lite
up-mock:       ## no Gazebo at all: GUI + planner on a built-in kinematic mock (lightest)
	bash scripts/run.sh mock
up-software:   ## like `up` but with software rendering (if the Gazebo window is black / crashes)
	bash scripts/preflight.sh full; docker compose -f docker-compose.yml -f compose.software.yml --profile vlm up --build
build:
	docker compose --profile vlm build
check:         ## staged self-test of Gazebo inside the container (run this first!)
	docker compose build sim
	docker compose run --rm --no-deps -e GZ_GUI=0 --entrypoint /opt/ur5/scripts/selfcheck.sh sim
test:          ## unit/integration tests (no Docker, no Gazebo needed; python3 + numpy + pillow + lxml)
	python3 -m unittest discover -s tests -v
down:
	docker compose --profile vlm down
logs:
	docker compose --profile vlm logs -f
clean:
	docker compose --profile vlm down -v --rmi local
