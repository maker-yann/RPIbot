#!/bin/bash

echo "Copy trace file [y/n]?"
read ANTWORT
if [ "$ANTWORT" == "y" ]
    then
	echo "Copy file"
	sshpass -p "raspberry" scp pi@rpibot:/home/pi/RPIbot/trace.csv .
    else
        echo "No file to copy."
fi

# Available:
# timestamp;pwmL;speedL;encoderR;I_L;Iyaw;odoDistance;CycleTimeControl;yaw;absYaw;yawCorr;I_R;hdg;m_y;m_x;ax;eYaw;speedR;encoderL;wz;pwmR;cycleTimeSense;

# Left complete
#python3 plotter.py -f trace.csv -t timestamp -p speedL,pwmL,encoderL

# Right complete
#python3 plotter.py -f trace.csv -t timestamp -p speedR,pwmR,encoderR

# Physics
#python3 plotter.py -f trace.csv -t timestamp -p eYaw,speedL,speedR,hdg,wz,m_x,m_y
#python3 plotter.py -f trace.csv -t timestamp -p yaw,odoDistance,absYaw,eYaw
#python3 plotter.py -f trace.csv -t timestamp -p yaw,odoDistance,encoderL,encoderR
python3 plotter.py -f trace.csv -t timestamp -p yaw,hdg,m_x,m_y,ax,wz

# Close loop control
#python3 plotter.py -f trace.csv -t timestamp -p eYaw,speedL,speedR,pwmL,pwmR,yawCorr
#python3 plotter.py -f trace.csv -t timestamp -p yaw,eYaw,yawCorr,speedL,speedR,eSpeedL,pwmL,pwmR,Iyaw,encoderL,encoderR
#python3 plotter.py -f trace.csv -t timestamp -p yawCorr,speedL,speedR,eSpeedL,eSpeedR,pwmL,pwmR,Iyaw,encoderL,encoderR
#python3 plotter.py -f trace.csv -t timestamp -p yawCorr,speedL,speedR,eSpeedL,eSpeedR,Iyaw,I_L,I_R
#python3 plotter.py -f trace.csv -t timestamp -p yawCorr,speedL,speedR,Iyaw,eYaw

# Open loop control
#python3 plotter.py -f trace.csv -t timestamp -p speedL,speedR,pwmL,pwmR

# Debug
#python3 plotter.py -f trace.csv -t timestamp -p CycleTimeControl,cycleTimeSense,odoDistance,yaw,encoderL
