#!/bin/bash
#
# embeds the target image into the XML config file and schedules a FlockLab test (ASAP)
#
# 2020, rdaforno
#

XMLFILE=flocklab_dpp2lora_elwb.xml
IMGFILE="Debug/comboard_elwb.elf"
OBSIDS=$(sed -nE 's/^#define ELWB_CONF_SCHED_NODE_LIST\s*([0-9, ]+).*/\1/p' Inc/app_config.h | sed s/,//g)
SEDCMD=sed
B64CMD=base64
FLTOOLS=flocklab

XMLTEMPLATE=$(cat <<- END
<?xml version="1.0" encoding="UTF-8"?>
<testConf xmlns="http://www.flocklab.ethz.ch" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://www.flocklab.ethz.ch xml/flocklab.xsd">
    <!-- General configuration -->
    <generalConf>
        <name>eLWB test</name>
        <description>
            A test of the eLWB on the DPP2 LoRa Comboard.
        </description>
        <schedule>
          <duration>65</duration>
        </schedule>
        <emailResults>no</emailResults>
    </generalConf>

    <!-- Target configuration -->
    <targetConf>
        <obsIds>2 3 4 5 6</obsIds>
        <voltage>3.3</voltage>
        <embeddedImageId>Image_1</embeddedImageId>
    </targetConf>

    <!-- Serial Service configuration -->
    <serialConf>
        <obsIds>2 3 4 5 6</obsIds>
        <baudrate>1000000</baudrate>
    </serialConf>

    <gpioTracingConf>
        <obsIds>2 3 4 5 6</obsIds>
        <pins>INT1 INT2 LED1 LED2 LED3</pins>
        <offset>1</offset>
    </gpioTracingConf>

    <!-- Power Profiling Service configuration -->
    <!--<powerProfilingConf>
        <obsIds>2 3 4 5 6</obsIds>
        <offset>0</offset>
        <duration>15</duration>
        <samplingRate>100</samplingRate>
    </powerProfilingConf>-->

    <embeddedImageConf>
        <embeddedImageId>Image_1</embeddedImageId>
        <name>eLWB 0.1</name>
        <description>eLWB test</description>
        <platform>dpp2lora</platform>
        <data>
        </data>
    </embeddedImageConf>
</testConf>
END
)


# check if sed tool is installed
which $SEDCMD > /dev/null 2>&1
if [ $? -ne 0 ]
then
  echo "command '$SEDCMD' not found"
  exit 1
fi

# check if base64 tool is installed
which $B64CMD > /dev/null 2>&1
if [ $? -ne 0 ]
then
  echo "command '$B64CMD' not found"
  exit 1
fi

# check if flocklab tools are installed
which $FLTOOLS > /dev/null 2>&1
if [ $? -ne 0 ]
then
  echo "command '$FLTOOLS' not found"
  exit 1
fi

# check if files exist
if [ ! -f $IMGFILE ]; then
  echo "file $IMGFILE not found"
  exit 1
fi
if [ ! -f $XMLFILE ]; then
  echo "$XMLTEMPLATE" > $XMLFILE
  echo "file $XMLFILE created"
fi

# convert to base 64
B64FILE="$IMGFILE.b64"
$B64CMD $IMGFILE > $B64FILE
# insert binary into xml (in-place)
$SEDCMD -i -n '1h;1!H;${ g;s/<data>.*<\/data>/<data>\n<\/data>/;p}' $XMLFILE
$SEDCMD -i "/<data>/r ${B64FILE}" $XMLFILE
# remove temporary file
rm $B64FILE

echo "Target image $IMGFILE embedded into $XMLFILE."

if [ ! -z "$OBSIDS" ]; then
  # insert observer list
  $SEDCMD -i "s/<obsIds>.*<\/obsIds>/<obsIds>$OBSIDS<\/obsIds>/g" $XMLFILE
  echo "Observer IDs inserted."
else
  # read observer list from file
  OBSIDS=$($SEDCMD -n 's/.*<obsIds>\(.*\)<\/obsIds>/\1/p' $XMLFILE | head -1)
fi

# validate the file
RES=$($FLTOOLS -v $XMLFILE)
if [[ $RES = *"validated correctly"* ]]; then
  echo "File validated correctly."
else
  echo "XML validation failed. $RES" | $SEDCMD 's/<li>/\n/g' | $SEDCMD 's/<[a-z\/]*>//g'
  exit 1
fi

# read the duration from the xml file
DURATION=$($SEDCMD -n 's/.*<duration>\(.*\)<\/duration>/\1/p' $XMLFILE | head -1)
echo "Scheduling FlockLab test... (observers: $OBSIDS, duration: ${DURATION}s)"
sleep 3   # give the user time to abort

# schedule the test
$FLTOOLS -c $XMLFILE
