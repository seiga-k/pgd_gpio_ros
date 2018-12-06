#include <ros/ros.h>
#include <ros/console.h>
#include <ros/xmlrpc_manager.h>
#include <pgd_gpio_ros/gpio.h>

#include <iostream>
#include <string>
#include <map>
#include <numeric>
#include <regex>

#include <pigpiod_if2.h>

class PgdGpio{
public:
	PgdGpio():
	nh("~")
	{
		XmlRpc::XmlRpcValue params;
		if(!nh.getParam(nh.getNamespace(), params)){
			ROS_ERROR("rcio must need valid parameters");
			ros::shutdown();
		}
		nh.param<std::string>("PigpiodIP", mpi_ip, "localhost");
		nh.param<std::string>("PigpiodPort", mpi_port, "8888");
		mpi = pigpio_start(const_cast<char*>(mpi_ip.c_str()), const_cast<char*>(mpi_port.c_str()));
		if(mpi < 0){
			ROS_ERROR("Pigpiod Error : %s", pigpio_error(mpi));
			ros::shutdown();
		}
		try{
			for(auto&& param : params){
                if(param.first == "input"){
                    std::cout << "Input settings" << std::endl;
                    if(param.second.hasMember("port")){
                        input_list.clear();
                        int32_t port_num(param.second["port"].size());
                        bool has_invert(check_member(param.second, "invert", port_num));
                        bool has_pull(check_member(param.second, "pull", port_num));
                        bool has_event(check_member(param.second, "event", port_num));
                        bool has_filter(check_member(param.second, "filter", port_num));

                        for(int i = 0; i < port_num; ++i){
                            input_list.push_back(param.second["port"][i]);
                            check_pgd_error(set_mode(mpi, input_list[i], PI_INPUT));
                            if(has_invert){
                                bool inv(param.second["invert"][i]);
                                //std::cout << "Set invert option" << std::endl;
                                input_inverts.push_back(inv);
                            }
                            if(has_pull){
                                //std::cout << "Set pull up down option" << std::endl;
                                int ret;
                                if(param.second["pull"][i] == "up"){
                                    ret = set_pull_up_down(mpi, input_list[i], PI_PUD_UP);
                                }else if(param.second["pull"][i] == "down"){
                                    ret = set_pull_up_down(mpi, input_list[i], PI_PUD_DOWN);
                                }else{
                                    ret = set_pull_up_down(mpi, input_list[i], PI_PUD_OFF);
                                }
                                check_pgd_error(ret);
                            }
                            if(has_event){
                                //std::cout << "Set event option" << std::endl;
                                int ret;
                                if(param.second["event"][i]){
                                    ret = callback_ex(mpi, input_list[i], EITHER_EDGE, in_cb, reinterpret_cast<void*>(this));
                                    check_pgd_error(ret);
                                    input_cb_id.push_back(ret);
                                }
                            }
                            if(has_filter){
                                //std::cout << "Set filter option" << std::endl;
                                int ret;
                                int us(param.second["filter"][i]);
                                ret = set_glitch_filter(mpi, input_list[i], us);
                                check_pgd_error(ret);
                            }
                        }

                        pub_in = nh.advertise<pgd_gpio_ros::gpio>("input", 10);
                    }else{
                        ROS_ERROR("Input need port element.");
                        ros::shutdown();
                    }
                }else if(param.first == "output"){
                    //std::cout << "Onput settings" << std::endl;
                    if(param.second.hasMember("port")){
                        output_list.clear();
                        int32_t port_num(param.second["port"].size());
                        bool has_invert(check_member(param.second, "invert", port_num));
                        bool has_default(check_member(param.second, "default", port_num));

                        for(int i = 0; i < port_num; i++){
                            output_list.push_back(param.second["port"][i]);
                            check_pgd_error(set_pull_up_down(mpi, output_list[i], PI_PUD_OFF));
                            if(has_invert){
                                bool inv(param.second["invert"][i]);
                                //std::cout << "Set invert option" << std::endl;
                                output_inverts[output_list[i]] = inv;
                            }
                            if(has_default){
                                int level(param.second["default"][i]);
                                int ret = gpio_write(mpi, output_list[i], level);
                                check_pgd_error(ret);
                            }
                            check_pgd_error(set_mode(mpi, output_list[i], PI_OUTPUT));
                        }

                        sub_out = nh.subscribe("output", 10, &PgdGpio::out_cb, this);
                    }else{
                        ROS_ERROR("Output need port element.");
                        ros::shutdown();
                    }
                }
			}
		}
		catch(std::invalid_argument& e){
			ROS_ERROR("Parameter parse error : %s", e.what());
			ros::shutdown();
		}
		catch(XmlRpc::XmlRpcException& e){
			ROS_ERROR("Parameter parse error. Code %d : %s", e.getCode(), e.getMessage().c_str());
			ros::shutdown();
		}
		if(output_list.size() == 0 && input_list.size() == 0){
			ROS_ERROR("No configurable ports.");
			ros::shutdown();
		}
	}
	
	~PgdGpio(){
		if(mpi >= 0){
            for(auto&& id : input_cb_id){
                callback_cancel(id);
            }
            for(auto&& gpio : output_list){
                set_mode(mpi, gpio, PI_INPUT);
            }
			pigpio_stop(mpi);
		}
	}
	
private:
	ros::NodeHandle nh;
	ros::Subscriber sub_out;
	ros::Publisher pub_in;
	std::vector<int32_t> output_list;
	std::map<int32_t, int32_t> output_inverts;
	std::vector<int32_t> input_list;
	std::vector<int32_t> input_inverts;
    std::vector<int32_t> input_cb_id;
	int32_t mpi;
	std::string mpi_ip;
	std::string mpi_port;
	
	void out_cb(const pgd_gpio_ros::gpio::ConstPtr msg){
        bool domask(false);
        if(msg->port_numbers.size() != msg->datas.size()){
            ROS_WARN("port array and data array length miss mutched");
            return;
        }
        if(msg->masks.size() == msg->port_numbers.size()){
            domask = true;
        }
        for(int i = 0; i < msg->port_numbers.size(); i++){
            if(domask){
                if(!msg->masks[i]){
                    if(std::count(output_list.cbegin(), output_list.cend(), msg->port_numbers[i]) == 1){
                        gpio_write(mpi, msg->port_numbers[i], (msg->datas[i] & 0x01) ^ output_inverts[msg->port_numbers[i]]);
                    }else{
                        ROS_WARN("Specified not configured output port");
                    }
                }
            }else{
                if(std::count(output_list.cbegin(), output_list.cend(), msg->port_numbers[i]) == 1){
                    gpio_write(mpi, msg->port_numbers[i], (msg->datas[i] & 0x01) ^ output_inverts[msg->port_numbers[i]]);
                }else{
                    ROS_WARN("Specified not configured output port");
                }
            }
        }
	}

	static void in_cb(int pi, unsigned int user_gpio, unsigned int level, unsigned int tick, void* userdata){
		PgdGpio* obj = reinterpret_cast<PgdGpio*>(userdata);
		obj->in_cb_proc(pi, user_gpio, level, tick);
	}

	inline void in_cb_proc(int pi, unsigned int user_gpio, unsigned int level, unsigned int tick){
        uint64_t port = ((uint64_t)read_bank_2(mpi) << 32) + (uint64_t)read_bank_1(mpi);
        port &= ~(1 << user_gpio);
        port |= (level << user_gpio);
        // 変化が早い信号の場合，read_bankした頃には信号レベルが変わってしまっているので，引数と組み合わせる．
        //std::printf("Bank : %X\r\n", port);
        pgd_gpio_ros::gpio gpio;
        gpio.header.stamp = ros::Time::now();
        for(int i = 0; i < input_list.size(); i++){
            gpio.port_numbers.push_back(input_list[i]);
            gpio.datas.push_back(((port >> input_list[i]) & 0x01) ^ input_inverts[i]);
        }
        pub_in.publish(gpio);
	}

    bool check_member(XmlRpc::XmlRpcValue& rhs, const std::string& name, const int32_t size){
        if(rhs.hasMember(name)){
            if(rhs[name].size() == size){
                return true;
            }
        }
        return false;
    }

    bool check_pgd_error(int code){
        if(code < 0){
            ROS_ERROR("Pigpiod Error. Code : %s", pigpio_error(code));
            ros::shutdown();
            return false;
        }
        return true;
    }
};

int main(int argc, char** argv){
	ros::init(argc, argv, "pgd_gpio_node");
    ROS_INFO("Start pgd_gpio_node");
	PgdGpio rcio;
	ros::spin();
	return 0;
}