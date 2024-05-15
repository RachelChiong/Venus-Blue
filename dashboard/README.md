# Dashboard
## Requirements
- docker
- docker-compose

## How to deploy
1. Ensure docker is running
2. Go to ```dashboard/``` and run
```sh
docker-compose up
```
This should spin up two sub-containers.
- frontend: ```localhost:3000```
- backend: ```localhost:5001```

3. To close the dashboard, run
```sh
docker-compose down -v
```

