#include <ros/ros.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <unistd.h> /* close() */
#include <v2x_udpbroadcast2/decode.h>
#include <v2x_msgs/v2x_info.h>

class Paramreader{
public:
	Paramreader(ros::NodeHandle& nh){
		nh.param("debug_asn", debug_asn, false);
	}
	bool Debug_asn() const {return debug_asn;}
private:
	bool debug_asn;
};

int main(int argc, char *argv[]){
	ros::init(argc, argv, "v2x_main");
	ros::NodeHandle nh;
	ros::Publisher v2x_pub = nh.advertise<v2x_msgs::v2x_info>("v2x_info", 10);
	Paramreader paramreader(nh);

    struct timeval tv;
    fd_set fds;

    struct sockaddr_in sockaddr;
	int sock = -1;
	int ret = -1;
    int on = 1;

	uint8_t buf[4096];
	int nrecv;

    memset(&sockaddr, 0, sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_port = htons(60000);
    sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == -1) 
        throw std::runtime_error("failed to create socket\n");

    ret = setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
    if (ret == -1) {
		close(sock);
        std::string err = std::string() +
            "failed to set socket option : " + strerror(errno);
		throw std::runtime_error(err.c_str());
    }

    ret = bind(sock, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
    if (ret == -1) {
        close(sock);
        std::string err = std::string() +
            "failed to bind socket: %d\n" + strerror(errno);
		throw std::runtime_error(err.c_str());
    }
	
	v2x_msgs::v2x_info msg;
	MessageFrame_t *frame = nullptr;
	while (ros::ok()) {
		FD_ZERO(&fds);
		FD_SET(sock, &fds);

		tv.tv_sec  = 0;
		tv.tv_usec = 1000; /* 1ms */
		ret = select(sock + 1, &fds, NULL, NULL, &tv);
		if (ret == -1) {
			if (errno == EINTR)
				continue;
            close(sock);
            std::string err = std::string() +
                "failed to select socket: %d\n" + strerror(errno);
		    throw std::runtime_error(err.c_str());
		}
		else if (ret == 0)
			continue;

		if (FD_ISSET(sock, &fds)) {
			nrecv = recvfrom(sock, buf, sizeof(buf), 0, NULL, NULL);
			if (nrecv == -1) {
                std::string err = std::string() +
                    "failed to recvfrom: %d\n" + strerror(errno);
		        throw std::runtime_error(err.c_str());
			}
			decode_avc2019(buf, nrecv, frame, &msg);
			v2x_pub.publish(msg);
			if(paramreader.Debug_asn()) 
				asn_fprint(stdout, &asn_DEF_MessageFrame, frame);
		}
	}

	close(sock);
	return EXIT_SUCCESS;
}