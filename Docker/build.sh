#!/bin/bash

# Author: مهدي بلال

docker-compose -f compose.yaml build $(docker-compose -f compose.yaml config --services)