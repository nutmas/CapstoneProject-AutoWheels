#!/bin/bash
#docker kill jupyter
#docker rm jupyter
#docker run --rm --name=jupyter -p 8888:8888 -w="/capstone" -v $PWD:/capstone --rm -it capstone jupyter notebook --no-browser --ip=0.0.0.0 --allow-root

jupyter notebook --no-browser --ip=0.0.0.0 --port=8888
