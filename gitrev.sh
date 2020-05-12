#!/bin/bash
# writes the GIT hash and compile date into the file gitrev.h
GITSHA=$(git rev-parse HEAD | head -c 8)
GITSHA_INT32=$(echo $(( 16#$GITSHA)))
CURRTIME=$(date +%s)
printf "\n#ifndef __GITREV_H\n#define __GITREV_H\n\n#define GIT_REV       \"${GITSHA}\"\n#define GIT_REV_INT   ${GITSHA_INT32}\n#define BUILD_TIME    ${CURRTIME}\n\n#endif /* __GITREV_H */" > ../Inc/gitrev.h
