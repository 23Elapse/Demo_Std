#include "tsk_wifi.h"


QueueHandle_t cmdQueue;

void vWifiTask(void *pvParameters) {
    char cmdBuffer[64];
    while (1) {
        // 从队列获取指令
        if (xQueueReceive(cmdQueue, cmdBuffer, portMAX_DELAY) == pdPASS) {
            // 发送AT指令
            USART_SendString(USART1, cmdBuffer);
            USART_SendString(USART1, "\r\n");
            
            // 等待响应（示例：等待"OK"）
            char response[16];
            USART_ReceiveString(USART1, response, 16);
            if (strstr(response, "OK") != NULL) {
                // 处理成功响应
            }
        }
    }
}


#if 0
#define TCP_CLIENT_TASK_STACK_SIZE 512
void vTcpClientTask(void *pvParameters) {
    struct netconn *conn;
    err_t err;
    
    // 创建TCP连接（示例：连接到服务器IP:192.168.1.100，端口80）
    conn = netconn_new(NETCONN_TCP);
    netconn_bind(conn, NULL, 0);
    err = netconn_connect(conn, &ipaddr, 80);
    
    if (err == ERR_OK) {
        char *message = "GET / HTTP/1.1\r\nHost: 192.168.1.100\r\n\r\n";
        netconn_send(conn, message, strlen(message));
        
        // 接收数据
        char buf[128](@ref);
        netbuf *nb = netconn_recv(conn);
        while (nb != NULL) {
            netbuf_data(nb, (void**)&buf, &len);
            // 处理接收数据
            netbuf_delete(nb);
        }
        netconn_close(conn);
        netconn_delete(conn);
    }
}
#endif

