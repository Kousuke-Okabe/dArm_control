COMMAND

rostopic pub -1 /tcp_command std_msgs/Float64MultiArray "{layout: {dim: [],data_offset: 0},data: [x[mm], y[mm], eta[rad]]}"