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

#define BROKER_HOSTNAME     "192.168.31.225"   // 192.168.8.101
#define TEST_TOPIC          "dev/test"

void messageArrived(MessageData *data)
{
    printf("Message arrived on topic %.*s: %.*s\n", data->topicName->lenstring.len, data->topicName->lenstring.data,
           data->message->payloadlen, data->message->payload);
}

void prvMQTTEchoTask(void *pvParameters)
{
    /* connect to m2m.eclipse.org, subscribe to a topic, send and receive messages regularly every 1 sec */
    MQTTClient client;
    Network network;
    unsigned char sendbuf[80], readbuf[80];
    int rc = 0,
        count = 0;
    MQTTPacket_connectData connectData = MQTTPacket_connectData_initializer;

    pvParameters = 0;
    NetworkInit(&network);
    MQTTClientInit(&client, &network, 1000, sendbuf, sizeof(sendbuf), readbuf, sizeof(readbuf));

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
    connectData.clientID.cstring = "FreeRTOS_sample";
    connectData.username.cstring = "billy";
    connectData.password.cstring = "password";

    if ((rc = MQTTConnect(&client, &connectData)) != 0)
        printf("Return code from MQTT connect is %d\n", rc);
    else
        printf("MQTT Connected\n");
#if 1
    if ((rc = MQTTSubscribe(&client, TEST_TOPIC, 2, messageArrived)) != 0)
        printf("Return code from MQTT subscribe is %d\n", rc);
    printf("MQTT sub successful\r\n");
#endif
    while (++count)
    {
        MQTTMessage message;
        char payload[30];

        message.qos = 1;
        message.retained = 0;
        message.payload = payload;
        sprintf(payload, "message number %d", count);
        message.payloadlen = strlen(payload);

        if ((rc = MQTTPublish(&client, TEST_TOPIC, &message)) != 0) {
            printf("Return code from MQTT publish is %d\n", rc);
            break;
        }
        printf("MQTT publish successful\r\n");
        vTaskDelay(2000);
#if !defined(MQTT_TASK)
        if ((rc = MQTTYield(&client, 1000)) != 0)
            printf("Return code from yield is %d\n", rc);
#endif
    }

    /* do not return */
    for (;;);
}

void vStartMQTTTasks(uint16_t usTaskStackSize, UBaseType_t uxTaskPriority)
{
    BaseType_t x = 0L;

    xTaskCreate(prvMQTTEchoTask, /* The function that implements the task. */
                "MQTTEcho0",     /* Just a text name for the task to aid debugging. */
                usTaskStackSize, /* The stack size is defined in FreeRTOSIPConfig.h. */
                (void *)x,       /* The task parameter, not used in this case. */
                uxTaskPriority,  /* The priority assigned to the task is defined in FreeRTOSConfig.h. */
                NULL);           /* The task handle is not used. */
}
/*-----------------------------------------------------------*/
