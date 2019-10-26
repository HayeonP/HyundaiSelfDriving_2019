#include <nodelet_lidar_driver/nodelet_frontend.h>

namespace LidarNodeletFront{
  void LidarNodeletFront::extMsg(){
    while(ros::ok()){
      //void LidarNodeletFront::timerCallback(const ros::TimerEvent& event){
      ip_pkt ip_pkt;
      std::string dst_ipaddr;
      ip_pkthdr ip_hdr;
      udp_pkthdr udp_hdr;    
      int frag_offset, mf_flag, dst_port;

      if(socket_flag == true) {   
        while(true) {
  	      ret = read_from_socket(sockfd, ip_pkt, sender_address);
  	      dst_ipaddr = ipaddr_from_sockaddr(sender_address);
  	      if(ret == 0) break;    
        }
        get_ip_header(ip_pkt, ip_hdr);
      }
      else {
        if ((ret = read_pcap_file(pcap_fd, ip_pkt)) == false) {
  	      ROS_ERROR("cannot read pcap file");
  	      exit(1);
        }
        get_ip_header(ip_pkt, ip_hdr);
        if ((frag_offset = ip_hdr.flag_n_frag & 0x0fff) == 0) {
  	      get_udp_header(ip_pkt.buf, udp_hdr);
  	      sender_address.sin_addr.s_addr = ip_hdr.src_addr;
  	      dst_ipaddr = inet_ntoa(sender_address.sin_addr);
  	      dst_port = udp_hdr.dst_port;
        }
        if (mf_flag = ((ip_hdr.flag_n_frag & 0x2000) >> 13)) { 
      	  OSI64::get_OSI64_msg(socket_flag, ip_pkt, lidar_msg, frag_offset); // current packet assembled
          continue;
        }
      }
      if (dst_ipaddr == param.ip_addr[0]) {
        frag_offset = 0;
        OSI64::get_OSI64_msg(socket_flag, ip_pkt, lidar_msg, frag_offset);
        // VLP16::get_VLP16_msg(socket_flag, ip_pkt, lidar_msg);
        pub_msg[0].publish(lidar_msg);
      }
      else if(dst_ipaddr == param.ip_addr[1]) {
        // VLP16::get_VLP16_msg(socket_flag, ip_pkt, lidar_msg);
        // pub_msg[1].publish(lidar_msg);
      }
      else if(dst_ipaddr == param.ip_addr[2]) {
        // VLP16::get_VLP16_msg(socket_flag, ip_pkt, lidar_msg);
        // pub_msg[2].publish(lidar_msg);
      }
      else{
        ROS_WARN("Unknown LIDAR!!");
      }
    }
  }
  void LidarNodeletFront::onInit(){
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    private_nh.getParam("mode", param.mode);
    private_nh.getParam("pcap_file", param.pcap_file);
    private_nh.getParam("ip_add_front", param.ip_addr[0]);
    private_nh.getParam("ip_add_left", param.ip_addr[1]);
    private_nh.getParam("ip_add_right", param.ip_addr[2]);

    if(param.pcap_file == ""){
      socket_flag = true;
      sockfd = create_socket();
    }
    else
      if ((ret = open_pcap_file(&pcap_fd, param.pcap_file)) == false) 
	      exit(1);

    if (param.mode == VLP_ONLY) {
      ROS_INFO("LIDAR MODE : VLP16 ONLY");
      pub_msg[0] = nh.advertise<nodelet_lidar_driver::VLP16>("front/lidar_msg", queue_size);
      pub_msg[1]  = nh.advertise<nodelet_lidar_driver::VLP16>("left/lidar_msg", queue_size);
      pub_msg[2] = nh.advertise<nodelet_lidar_driver::VLP16>("right/lidar_msg", queue_size);
    }
    else if (param.mode == OSI_ONLY) {
      ROS_INFO("LIDAR MODE : OS-64 ONLY");
      pub_msg[0] = nh.advertise<nodelet_lidar_driver::OSI64>("front/lidar_msg", queue_size);
      // pub_msg[1]  = nh.advertise<nodelet_lidar_driver::OSI64>("left/lidar_msg", queue_size);
      // pub_msg[2] = nh.advertise<nodelet_lidar_driver::OSI64>("right/lidar_msg", queue_size);
    }
    else if(param.mode == VLP_OSI_FUSION) {
      ROS_INFO("LIDAR MODE : OS-64 + VLP16 FUSION");
      pub_msg[0] = nh.advertise<nodelet_lidar_driver::OSI64>("front/lidar_msg", queue_size);
      pub_msg[1]  = nh.advertise<nodelet_lidar_driver::VLP16>("left/lidar_msg", queue_size);
      pub_msg[2] = nh.advertise<nodelet_lidar_driver::VLP16>("right/lidar_msg", queue_size);
    }
    else {
      ROS_ERROR("Unknown mode");
      exit(1);
    }
    // 753.5 * 3개 패킷 ==> (753.5 packets/second)
    // ros::Rate loop_rate(10000);
    extMsg(); 
  }
}
