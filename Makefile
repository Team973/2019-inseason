downloadlogs:
	python3 ./tools/log/downloadlogs.py 973

clearlogs:
	./tools/log/clearlogs.sh

builddeploy:
	./gradlew build
	./gradlew deploy
