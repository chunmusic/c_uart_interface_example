all: git_submodule mavlink_control

mavlink_control: one_unit.cpp mavlink_control.cpp serial_port.cpp udp_port.cpp autopilot_interface.cpp
	g++ -g -Wall -I mavlink/include/mavlink/v2.0 mavlink_control.cpp serial_port.cpp udp_port.cpp autopilot_interface.cpp -o mavlink_control -lpthread
	g++ -g -Wall -I mavlink/include/mavlink/v2.0 one_unit.cpp serial_port.cpp udp_port.cpp autopilot_interface.cpp -o one_unit -lpthread

git_submodule:
	git submodule update --init --recursive

clean:
	 rm -rf *o mavlink_control
