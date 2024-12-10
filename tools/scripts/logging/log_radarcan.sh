candump -L can0 > $1_can0.log &
candump -L can1 > $1_can1.log &
candump -L can2 > $1_can2.log & 
candump -L can3 > $1_can3.log &
candump -L can4 > $1_can4.log &
candump -L can5 > $1_can5.log

kill $(jobs -p)
