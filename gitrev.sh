#!/bin/bash
#
# Copyright (c) 2020 - 2021, ETH Zurich, Computer Engineering Group (TEC)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
# OF THE POSSIBILITY OF SUCH DAMAGE.
#
#
# writes the GIT hash and compile date into the file gitrev.h
#

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
