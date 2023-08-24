#!/bin/bash

#open another interactive bash prompt of running container
#for now only works for one running container
#TODO: add autocompletion or variable input
docker exec -it $(docker ps -aq) /bin/bash 
