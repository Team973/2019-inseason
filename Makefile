.PHONY: build
.PHONY: all

build:
	reset && clear
	./gradlew build

all: build
	./gradlew deploy

downloadlogs:
	python3 ./tools/log/downloadlogs.py 973

clearlogs:
	./tools/log/clearlogs.sh
