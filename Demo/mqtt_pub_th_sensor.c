
/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "MQTTClient.h"

#define BROKER_HOSTNAME     "192.168.8.101"   // 192.168.8.101
#define CLINET_ID           "PANDOR_TH_SENSOR_BOARD"
#define MQTT_USERNAME       "billy"
#define MQTT_PASSWD         "password"
#define TH_SENSOR_TOPIC     "dev/th_sensor"

#define MQTT_MSG_PALOAD_BUF_LEN (50)

extern QueueHandle_t th_sensor_msg;
extern uint16_t th_sensor_msg_buf[2];

void thSensorMsgArrived(MessageData *data)
{
    printf("Message arrived on topic %.*s: %.*s\n", data->topicName->lenstring.len, data->topicName->lenstring.data,
           data->message->payloadlen, data->message->payload);
}

void mqttTHSensorTask(void *argv)
{
    (void)argv;
    MQTTClient client;
    Network network;
    unsigned char sendbuf[80], readbuf[80];
    int rc = 0;
    MQTTPacket_connectData connectData = MQTTPacket_connectData_initializer;

    NetworkInit(&network);
    MQTTClientInit(&client, &network, 30000, sendbuf, sizeof(sendbuf), readbuf, sizeof(readbuf));

    char *address = BROKER_HOSTNAME; 
    if ((rc = NetworkConnect(&network, address, 1883)) != 0)
        printf("Return code from network connect is %d\n", rc);

    // FIXME: create MQTT task will mess up RTOS scheduler(seem to mutex deadlock)
#if 0
#if defined(MQTT_TASK)
    if ((rc = MQTTStartTask(&client)) != pdPASS)
        printf("Return code from start tasks is %d\n", rc);
#endif
#endif

    connectData.MQTTVersion = 3;
    connectData.clientID.cstring = CLINET_ID;
    connectData.username.cstring = MQTT_USERNAME;
    connectData.password.cstring = MQTT_PASSWD;

    // TODO: reconnet setup
    if ((rc = MQTTConnect(&client, &connectData)) != 0)
        printf("Return code from MQTT connect is %d\n", rc);
    else
        printf("MQTT Connected\n");

    // Debug message pub
    if ((rc = MQTTSubscribe(&client, TH_SENSOR_TOPIC, 2, thSensorMsgArrived)) != 0)
        printf("Return code from MQTT subscribe is %d\n", rc);
    printf("MQTT sub successful\r\n");

    while (1)
    {
        if (th_sensor_msg != 0) {
            if (xQueueReceive(th_sensor_msg, th_sensor_msg_buf, 0) == pdTRUE) {
                MQTTMessage message;
                float temp = th_sensor_msg_buf[0]/10.0-50;
                float humi = th_sensor_msg_buf[1]/10.0;
                char payload[MQTT_MSG_PALOAD_BUF_LEN];

                message.qos = 1;
                message.retained = 0;
                message.payload = payload;

                // FIXME: what is sensor message type?
                // sprintf(payload, "message number %d", count);
                snprintf(payload, MQTT_MSG_PALOAD_BUF_LEN-1, "temp=%.1f'C humi=%.1f%%", temp, humi);
                message.payloadlen = strlen(payload);

                if ((rc = MQTTPublish(&client, TH_SENSOR_TOPIC, &message)) != 0) {
                    printf("Return code from MQTT publish is %d\n", rc);
                }

                printf("MQTT publish successful\r\n");
            }
        }
    }
    
    /* do not return */
    for (;;);
}
/*-----------------------------------------------------------*/
