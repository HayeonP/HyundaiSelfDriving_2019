#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/select.h>

#include <MessageFrame.h>

#define J2735_UPER_MAP  18
#define J2735_UPER_SPAT 19
#define J2735_UPER_BSM  20
#define J2735_UPER_RSA  27
#define J2735_UPER_RTCM 28
#define J2735_UPER_TIM  31

typedef struct AVC_HDR {
	uint32_t sec;
	uint32_t usec;
	uint32_t type;
	uint32_t len;
} __attribute__((packed)) AVC_HDR_t;

typedef struct AVC_TAIL {
	uint16_t crc16;
} __attribute__((packed)) AVC_TAIL_t;

static int RUNNING = 0;

static void decode_MapData(MessageFrame_t *frame)
{
	// msg = &frame->value.MapData;
}

static void decode_SignalPhaseAndTimingMessage(MessageFrame_t *frame)
{
	// msg = &frame->value.SPAT;
}

static void decode_BasicSafetyMessage(MessageFrame_t *frame)
{
	BasicSafetyMessage_t *msg = &frame->value.BasicSafetyMessage;
    BSMcoreData_t *coreData = &msg->coreData;

    fprintf(stdout, "coreData\n");

    fprintf(stdout, "  msgCnt       : %ld\n", coreData->msgCnt);
    fprintf(stdout, "  id           : %02X.%02X.%02X.%02X\n",
			coreData->id.buf[0], coreData->id.buf[1],
			coreData->id.buf[2], coreData->id.buf[3]);
    fprintf(stdout, "  secMark      : %ld\n", coreData->secMark);
    fprintf(stdout, "  lat          : %ld\n", coreData->lat);
    fprintf(stdout, "  Long         : %ld\n", coreData->Long);
    fprintf(stdout, "  elev         : %ld\n", coreData->elev);

    fprintf(stdout, "  accuracy\n");
    fprintf(stdout, "    semiMajor  : %ld\n", coreData->accuracy.semiMajor);
    fprintf(stdout, "    semiMinor  : %ld\n", coreData->accuracy.semiMinor);
    fprintf(stdout, "    orientation: %ld\n", coreData->accuracy.orientation);

    fprintf(stdout, "  transmission : %ld\n", coreData->transmission);
    fprintf(stdout, "  speed        : %ld\n", coreData->speed);
    fprintf(stdout, "  heading      : %ld\n", coreData->heading);
    fprintf(stdout, "  angle        : %ld\n", coreData->angle);

    fprintf(stdout, "  accelSet\n");
    fprintf(stdout, "    Long       : %ld\n", coreData->accelSet.Long);
    fprintf(stdout, "    lat        : %ld\n", coreData->accelSet.lat);
    fprintf(stdout, "    vert       : %ld\n", coreData->accelSet.vert);
    fprintf(stdout, "    yaw        : %ld\n", coreData->accelSet.yaw);

    fprintf(stdout, "  brakes\n");
    fprintf(stdout, "    wheelBrakes: %02X\n", coreData->brakes.wheelBrakes.buf[0]);
    fprintf(stdout, "    traction   : %ld\n", coreData->brakes.traction);
    fprintf(stdout, "    albs       : %ld\n", coreData->brakes.albs);
    fprintf(stdout, "    scs        : %ld\n", coreData->brakes.scs);
    fprintf(stdout, "    brakeBoos  : %ld\n", coreData->brakes.brakeBoost);
    fprintf(stdout, "    auxBrakes  : %ld\n", coreData->brakes.auxBrakes);

    fprintf(stdout, "  size\n");
    fprintf(stdout, "    width      : %ld\n", coreData->size.width);
    fprintf(stdout, "    length     : %ld\n", coreData->size.length);

	if (msg->partII == NULL || msg->partII->list.count == 0)
		return;

    fprintf(stdout, "partII\n");

	for (int i = 0; i < msg->partII->list.count; i++) {
		if (msg->partII->list.array[i]->partII_Id ==
				PartIIcontent__partII_Value_PR_VehicleSafetyExtensions &&
			msg->partII->list.array[i]->partII_Value.present ==
				PartIIcontent__partII_Value_PR_VehicleSafetyExtensions) {

			VehicleSafetyExtensions_t *vehicleSafetyExtensions =
				&msg->partII->list.array[i]->partII_Value.VehicleSafetyExtensions;
		}
	}
}

static void decode_RTCMcorrections(MessageFrame_t *frame)
{
	// msg = &message->RTCMcorrections;
}

static void decode_TravelerInformation(MessageFrame_t *frame)
{
	// msg = &frame->value.TravelerInformation;
}

static int decode_MessageFrame(const uint8_t *buf, int len)
{
	MessageFrame_t *frame = NULL;
	asn_dec_rval_t rval;

	rval = uper_decode_complete(NULL, &asn_DEF_MessageFrame,
			(void **)&frame, buf, len);
	if (rval.code != RC_OK) {
		fprintf(stderr, "failed to decode MessageFrame\n");
		fprintf(stdout, "================================================\n");
		ASN_STRUCT_FREE(asn_DEF_MessageFrame, frame);
		return -1;
	}

	fprintf(stdout, "------------------------------------------------\n");
	fprintf(stdout, "decoded %d, code: %d\n", (int)rval.consumed, rval.code);
	fprintf(stdout, "------------------------------------------------\n");
	asn_fprint(stdout, &asn_DEF_MessageFrame, frame);
	fprintf(stdout, "================================================\n");

	if (frame->messageId == J2735_UPER_MAP &&
			frame->value.present == MessageFrame__value_PR_MapData) {
		decode_MapData(frame);
	}
	else if (frame->messageId == J2735_UPER_SPAT &&
			frame->value.present == MessageFrame__value_PR_SPAT) {
		decode_SignalPhaseAndTimingMessage(frame);
	}
	else if (frame->messageId == J2735_UPER_BSM &&
			frame->value.present == MessageFrame__value_PR_BasicSafetyMessage) {
		decode_BasicSafetyMessage(frame);
	}
	else if (frame->messageId == J2735_UPER_RTCM &&
			frame->value.present == MessageFrame__value_PR_RTCMcorrections) {
		decode_RTCMcorrections(frame);
	}
	else if (frame->messageId == J2735_UPER_TIM &&
			frame->value.present == MessageFrame__value_PR_TravelerInformation) {
		decode_TravelerInformation(frame);
	}

	ASN_STRUCT_FREE(asn_DEF_MessageFrame, frame);
	return rval.code;
}

static int decode_avc2019(const uint8_t *buf, int len)
{
	if (len < sizeof(AVC_HDR_t)) {
		fprintf(stderr, "packet size is less than header\n");
		return -1;
	}

	const AVC_HDR_t *hdr = (AVC_HDR_t *)buf;

	uint32_t sec  = ntohl(hdr->sec);
	uint32_t usec = ntohl(hdr->usec);
	uint32_t type = ntohl(hdr->type);
	uint32_t dlen = ntohl(hdr->len);

	fprintf(stdout, "================================================\n");
	fprintf(stdout, "time: %u.%06u, type: %2u, len: %4u\n", sec, usec, type, dlen);

	if (dlen + sizeof(AVC_HDR_t) + sizeof(AVC_TAIL_t) != len) {
		fprintf(stderr, "packet size is less than data: %lu %u\n",
				dlen + sizeof(AVC_HDR_t), len);
		fprintf(stdout, "================================================\n");
		return -1;
	}

	return decode_MessageFrame(buf + sizeof(AVC_HDR_t), dlen);
}

static void signal_handler(int signum)
{
	fprintf(stdout, "\n");
    RUNNING = 0;
}

int main(int argc, char *argv[])
{
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
    if (sock == -1) {
        fprintf(stderr, "failed to create socket\n");
		return EXIT_FAILURE;
    }

    ret = setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
    if (ret == -1) {
        fprintf(stderr, "failed to set socket option: %d\n", errno);
		close(sock);
		return EXIT_FAILURE;
    }

    ret = bind(sock, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
    if (ret == -1) {
        fprintf(stderr, "failed to bind socket: %d\n", errno);
		close(sock);
		return EXIT_FAILURE;
    }

	signal(SIGINT, signal_handler);

	RUNNING = 1;

	while (RUNNING) {
		FD_ZERO(&fds);
		FD_SET(sock, &fds);

		tv.tv_sec  = 0;
		tv.tv_usec = 1000; /* 1ms */

		ret = select(sock + 1, &fds, NULL, NULL, &tv);
		if (ret == -1) {
			if (errno == EINTR)
				continue;
			fprintf(stderr, "failed to select socket: %d\n", errno);
			close(sock);
			return EXIT_FAILURE;
		}
		else if (ret == 0)
			continue;

		if (FD_ISSET(sock, &fds)) {
			nrecv = recvfrom(sock, buf, sizeof(buf), 0, NULL, NULL);
			if (nrecv == -1) {
				fprintf(stderr, "failed to recvfrom: %d\n", nrecv);
				close(sock);
				return EXIT_FAILURE;
			}

			ret = decode_avc2019(buf, nrecv);
			if (ret < 0) {
				fprintf(stderr, "failed to decode: %d\n", ret);
				close(sock);
				return EXIT_FAILURE;
			}
		}
	}

	close(sock);
	return EXIT_SUCCESS;
}

