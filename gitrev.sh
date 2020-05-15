#!/bin/bash
# writes the GIT hash and compile date into the file gitrev.h
OUTFILE="../Inc/gitrev.h"
GITSHA=$(git rev-parse HEAD | head -c 8)
GITSHA_INT32=$(echo $(( 16#$GITSHA)))
# get the current timestamp, with hourly granularity (to prevent frequent recompilation of the code due to changing timestamp)
CURRTIME=$(date +%s -d "$(date +'%F %H:00:00')")
# only update the file if the content has changed
NEWCONTENT="\n#ifndef __GITREV_H\n#define __GITREV_H\n\n#define GIT_REV       \"${GITSHA}\"\n#define GIT_REV_INT   ${GITSHA_INT32}\n#define BUILD_TIME    ${CURRTIME}\n\n#endif /* __GITREV_H */"
grep "${GITSHA}" $OUTFILE > /dev/null && grep "${CURRTIME}" $OUTFILE > /dev/null
if [ $? -ne 0 ]; then
  printf "$NEWCONTENT" > $OUTFILE
  echo "file $OUTFILE updated"
else
  echo "file content unchanged"
fi
