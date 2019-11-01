location=$1

cd $location

dates=($(ls -d */))

for ((i=0; i<${#dates[@]}; i++))
do
  cd ${dates[$i]}
  cameras=($(ls -d */))
  for ((j=0; j<${#cameras[@]}; j++))
  do
    if [ -d "${cameras[$j]}Contrast" ]
    then
      rm -r "${cameras[$j]}Contrast"
    fi

    if ! [ -d "${cameras[$j]}frames" ]
    then
      mkdir "${cameras[$j]}frames"
    fi
      mv ${cameras[$j]}frame_* "${cameras[$j]}frames"
      mv "${cameras[$j]}img_tstamps.csv" "${cameras[$j]}frames"
  done
  cd ..
done
