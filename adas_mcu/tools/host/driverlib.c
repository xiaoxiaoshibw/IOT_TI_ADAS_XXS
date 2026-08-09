#include "driverlib.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/can.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#define HOST_CAN_OBJECTS 32U
#define HOST_CAN_QUEUE_DEPTH 32U

typedef struct {
    uint16_t data[8];
} HostCanPayload;

static int g_can_fd = -1;
static uint32_t g_object_ids[HOST_CAN_OBJECTS];
static HostCanPayload g_rx_queue[HOST_CAN_OBJECTS][HOST_CAN_QUEUE_DEPTH];
static uint16_t g_rx_head[HOST_CAN_OBJECTS];
static uint16_t g_rx_count[HOST_CAN_OBJECTS];

static void host_fail(const char *operation)
{
    fprintf(stderr, "MCU SIL CAN %s failed: %s\n", operation, strerror(errno));
    exit(2);
}

static void host_open_socket(void)
{
    const char *interface_name = getenv("ADAS_MCU_CAN_INTERFACE");
    struct ifreq request;
    struct sockaddr_can address;

    if (interface_name == NULL || interface_name[0] == '\0') {
        interface_name = "vcan0";
    }
    g_can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (g_can_fd < 0) host_fail("socket");

    memset(&request, 0, sizeof(request));
    strncpy(request.ifr_name, interface_name, IFNAMSIZ - 1U);
    if (ioctl(g_can_fd, SIOCGIFINDEX, &request) < 0) host_fail("interface lookup");

    memset(&address, 0, sizeof(address));
    address.can_family = AF_CAN;
    address.can_ifindex = request.ifr_ifindex;
    if (bind(g_can_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        host_fail("bind");
    }

    int flags = 0;
    if ((flags = fcntl(g_can_fd, F_GETFL, 0)) < 0 ||
        fcntl(g_can_fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        host_fail("nonblocking setup");
    }
}

static uint16_t object_for_id(uint32_t can_id)
{
    uint16_t object;
    for (object = 0U; object < HOST_CAN_OBJECTS; object++) {
        if (g_object_ids[object] == can_id) return object;
    }
    return HOST_CAN_OBJECTS;
}

static void queue_frame(uint16_t object, const struct can_frame *frame)
{
    uint16_t tail;
    if (object >= HOST_CAN_OBJECTS || frame->can_dlc != 8U) return;
    if (g_rx_count[object] >= HOST_CAN_QUEUE_DEPTH) {
        g_rx_head[object] = (uint16_t)((g_rx_head[object] + 1U) %
                                       HOST_CAN_QUEUE_DEPTH);
        g_rx_count[object]--;
    }
    tail = (uint16_t)((g_rx_head[object] + g_rx_count[object]) %
                      HOST_CAN_QUEUE_DEPTH);
    for (uint16_t i = 0U; i < 8U; i++) {
        g_rx_queue[object][tail].data[i] = frame->data[i];
    }
    g_rx_count[object]++;
}

static void drain_socket(void)
{
    struct can_frame frame;
    ssize_t received;
    while (g_can_fd >= 0) {
        received = recv(g_can_fd, &frame, sizeof(frame), MSG_DONTWAIT);
        if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return;
        if (received != (ssize_t)sizeof(frame)) {
            if (received < 0 && errno == EINTR) continue;
            return;
        }
        if ((frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0U) {
            continue;
        }
        queue_frame(object_for_id(frame.can_id & CAN_SFF_MASK), &frame);
    }
}

void CAN_setupMessageObject(uint32_t base, uint16_t object,
                            uint32_t can_id, uint16_t frame_type,
                            uint16_t object_type, uint32_t msg_id_mask,
                            uint32_t flags, uint16_t dlc)
{
    (void)base;
    (void)frame_type;
    (void)object_type;
    (void)msg_id_mask;
    (void)flags;
    (void)dlc;
    if (object < HOST_CAN_OBJECTS) g_object_ids[object] = can_id;
}

void CAN_initModule(uint32_t base)
{
    (void)base;
    memset(g_object_ids, 0, sizeof(g_object_ids));
    memset(g_rx_head, 0, sizeof(g_rx_head));
    memset(g_rx_count, 0, sizeof(g_rx_count));
    host_open_socket();
}

void CAN_setBitRate(uint32_t base, uint32_t sysclk_hz,
                    uint32_t bitrate, uint16_t bit_time)
{
    (void)base;
    (void)sysclk_hz;
    (void)bitrate;
    (void)bit_time;
}

void CAN_enableRetry(uint32_t base) { (void)base; }
void CAN_startModule(uint32_t base) { (void)base; }
uint32_t CAN_getStatus(uint32_t base) { (void)base; return 0U; }

bool CAN_readMessage(uint32_t base, uint16_t object, uint16_t *data)
{
    (void)base;
    drain_socket();
    if (object >= HOST_CAN_OBJECTS || data == NULL || g_rx_count[object] == 0U) {
        return false;
    }
    for (uint16_t i = 0U; i < 8U; i++) {
        data[i] = g_rx_queue[object][g_rx_head[object]].data[i];
    }
    g_rx_head[object] = (uint16_t)((g_rx_head[object] + 1U) % HOST_CAN_QUEUE_DEPTH);
    g_rx_count[object]--;
    return true;
}

void CAN_sendMessage(uint32_t base, uint16_t object, uint16_t dlc,
                     const uint16_t *data)
{
    struct can_frame frame;
    if (base != 0U || object >= HOST_CAN_OBJECTS || data == NULL || dlc != 8U) return;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = g_object_ids[object];
    frame.can_dlc = 8U;
    for (uint16_t i = 0U; i < 8U; i++) frame.data[i] = (uint8_t)(data[i] & 0xFFU);
    if (g_can_fd < 0 || send(g_can_fd, &frame, sizeof(frame), MSG_DONTWAIT) !=
                         (ssize_t)sizeof(frame)) {
        host_fail("send");
    }
}
