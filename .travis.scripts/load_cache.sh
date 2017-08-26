#!/bin/bash

set -o errexit
set -o verbose

echo ${DOCKER_CACHE_FILE}

if [ -f ${DOCKER_CACHE_FILE} ]; then
	bunzip2 ${DOCKER_CACHE_FILE} –stdout | docker load || true
fi

