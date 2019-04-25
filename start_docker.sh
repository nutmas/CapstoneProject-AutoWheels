#!/bin/bash
docker kill udacity
docker rm udacity
docker run --rm --name=udacity -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
