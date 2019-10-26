#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/time.h>

#include <netinet/if_ether.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <arpa/inet.h>
#include <signal.h>

#include <pcap.h>


static int RUNNING = 0;

static void signal_handler(int signum)
{
	fprintf(stdout, "\n");
    RUNNING = 0;
}

static int dump_pkt(const unsigned char *pkt, struct timeval ts,
		unsigned int len, uint8_t *data, int *dlen)
{
	struct ip *ip;
	uint32_t iplen;

	struct udphdr *udp;
	uint32_t udplen;

	if (len < sizeof(struct ether_header)) {
		fprintf(stdout, "aaaaaaaa1");
		return -1;
	}
	pkt += sizeof(struct ether_header);
	len -= sizeof(struct ether_header);

	ip = (struct ip *)pkt;
	iplen = ip->ip_hl << 2;

	if (len < iplen)
		fprintf(stdout, "aaaaaaaa2");

	if (ip->ip_p != IPPROTO_UDP)
		return -1;

	pkt += iplen;
	len -= iplen;

	if (len < sizeof(struct udphdr))
		return -1;

	udp = (struct udphdr *)pkt;
	udplen = ntohs(udp->uh_ulen);

#if 1
	fprintf(stdout, "%ld.%06ld UDP src_port=%d dst_port=%d length=%d\n",
			ts.tv_sec, ts.tv_usec, ntohs(udp->uh_sport),
			ntohs(udp->uh_dport), ntohs(udp->uh_ulen));
#endif

	if (len < udplen)
		return -1;

	pkt += sizeof(struct udphdr);
	len -= sizeof(struct udphdr);

	memcpy(data, pkt, len);
	*dlen = len;

	return 0;
}

int main(int argc, char *argv[])
{
	pcap_t *pcap = NULL;
	char perr[PCAP_ERRBUF_SIZE];
	const unsigned char *pkt;
	struct pcap_pkthdr phdr;

	struct sockaddr_in sockaddr;
	int sock = -1;

	uint8_t data[4096];
	int dlen;
	int ret;

	if (argc != 3) {
		fprintf(stdout, "usage: [%s pcapfile ip_addr]\n", argv[0]);
		return EXIT_FAILURE;
	}

    memset(&sockaddr, 0, sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_port = htons(60000);
    // sockaddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    sockaddr.sin_addr.s_addr = inet_addr(argv[2]);


    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == -1) {
        fprintf(stderr, "failed to create socket\n");
        return EXIT_FAILURE;
    }

	pcap = pcap_open_offline(argv[1], perr);
	if (pcap == NULL) {
		fprintf(stderr, "failed to open pcapfile: %s\n", perr);
		return EXIT_FAILURE;
	}

	signal(SIGINT, signal_handler);

	RUNNING = 1;

	while (RUNNING) {
		pkt = pcap_next(pcap, &phdr);
		if (pkt == NULL)
			break;

		ret = dump_pkt(pkt, phdr.ts, phdr.caplen, data, &dlen);
		if (ret < 0) {
			fprintf(stderr, "failed to dump packet\n");
			continue;
		}

		ret = sendto(sock, data, dlen,
				0, (const struct sockaddr *)&sockaddr, sizeof(sockaddr));
		// ignore error
        // scanf("%*d");
        usleep(10000);
	}

	pcap_close(pcap);
	return EXIT_SUCCESS;

out:
	pcap_close(pcap);
	return EXIT_FAILURE;
}

