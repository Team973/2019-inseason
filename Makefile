.PHONY: build
.PHONY: all

build: clearterm
	./gradlew build

all: build
	./gradlew deploy

clearterm:
ifeq ($(UNAME),Darwin)
	/usr/bin/osascript -e 'tell application "System Events" to tell process "Terminal" to keystroke "k" using command down'
else
	clear && reset
endif

list:
	@grep '^[^#[:space:]|.].*:' Makefile

downloadlogs:
	python3 ./tools/log/downloadlogs.py 973

clearlogs:
	./tools/log/clearlogs.sh
