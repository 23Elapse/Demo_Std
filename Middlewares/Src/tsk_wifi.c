#include "tsk_wifi.h"

QueueHandle_t cmdQueue;

// void vWifiTask(void *pvParameters) {
//     char cmdBuffer[64];
//     while (1) {
//         // 从队列获取指令
//         if (xQueueReceive(cmdQueue, cmdBuffer, portMAX_DELAY) == pdPASS) {
//             // 发送AT指令
//             USART_SendString(USART1, cmdBuffer);
//             USART_SendString(USART1, "\r\n");
            
//             // 等待响应（示例：等待"OK"）
//             char response[16];
//             USART_ReceiveString(USART1, response, 16);
//             if (strstr(response, "OK") != NULL) {
//                 // 处理成功响应
//             }
//         }
//     }
// }


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



// static const AT_Cmd_Config at_cmd_table[] = {
//     // --------------- Basic Commands ---------------
//     {
//         .at_cmd = "AT\r\n",
//         .expected_resp = "OK",
//         .timeout_ms = 1000,
//         .retries = 3,
//         .description = "AT test"
//     },
//     {
//         .at_cmd = "AT+GMR\r\n",
//         .expected_resp = "OK",
//         .timeout_ms = 2000,
//         .retries = 2,
//         .description = "Query firmware version"
//     },
    
//     // --------------- WiFi Configuration ---------------
//     {
//         .at_cmd = "AT+CWMODE=1\r\n",
//         .expected_resp = "OK",
//         .timeout_ms = 1500,
//         .retries = 2,
//         .description = "Set WiFi mode (Station)"
//     },
//     {
//         .at_cmd = "AT+CWDHCP=1,1\r\n", 
//         .expected_resp = "OK",
//         .timeout_ms = 2000,
//         .retries = 2,
//         .description = "Enable DHCP client"
//     },
//     {
//         .at_cmd = "AT+CWJAP=\"Your_SSID\",\"Your_Password\"\r\n",
//         .expected_resp = "GOT IP",
//         .timeout_ms = 15000,
//         .retries = 5,
//         .description = "Connect to WiFi AP"
//     },
//     {
//         .at_cmd = "AT+CIPSTA?\r\n",
//         .expected_resp = "+CIPSTA:",
//         .timeout_ms = 2000,
//         .retries = 1,
//         .description = "Query IP address"
//     },
//     {
//         .at_cmd = "AT+CWJAP?\r\n",
//         .expected_resp = "OK",
//         .timeout_ms = 2000,
//         .retries = 2,
//         .description = "Query connection status"
//     },
//     {
//         .at_cmd = "AT+CIPMUX=0\r\n",
//         .expected_resp = "OK",
//         .timeout_ms = 2000,
//         .retries = 2,
//         .description = "Set single connection mode"
//     },
//     {
//         .at_cmd = "AT+CIPMODE=1\r\n",
//         .expected_resp = "OK",
//         .timeout_ms = 2000,
//         .retries = 2,
//         .description = "Set transparent mode"
//     },
//     {
//         .at_cmd = "AT+CIPSTO=10\r\n",
//         .expected_resp = "OK",
//         .timeout_ms = 2000,
//         .retries = 2,
//         .description = "Set timeout"
//     },

    
//     // --------------- TCP/IP Communication ---------------
//     {
//         .at_cmd = "AT+CIPSTART=\"TCP\",\"api.example.com\",80\r\n",
//         .expected_resp = "CONNECT",
//         .timeout_ms = 10000,
//         .retries = 3,
//         .description = "Establish TCP connection"
//     },
//     {
//         .at_cmd = "AT+CIPSEND=%d\r\n",
//         .expected_resp = ">",
//         .timeout_ms = 5000,
//         .retries = 2,
//         .description = "Enter data sending mode"
//     },
//     {
//         .at_cmd = "GET / HTTP/1.1\r\nHost: api.example.com\r\n\r\n",
//         .expected_resp = "HTTP/1.1",
//         .timeout_ms = 10000,
//         .retries = 2,
//         .description = "Send HTTP request"
//     },
//     {
//         .at_cmd = "AT+CIPCLOSE\r\n",
//         .expected_resp = "CLOSED",
//         .timeout_ms = 3000,
//         .retries = 2,
//         .description = "Close TCP connection"
//     },
    
//     // --------------- Advanced Features ---------------
//     {
//         .at_cmd = "AT+PING=\"www.google.com\"\r\n",
//         .expected_resp = "ms",
//         .timeout_ms = 10000,
//         .retries = 1,
//         .description = "Network ping test"
//     },
//     {
//         .at_cmd = "AT+CIPSNTPCFG=1,8,\"pool.ntp.org\"\r\n",
//         .expected_resp = "OK",
//         .timeout_ms = 5000,
//         .retries = 2,
//         .description = "Configure NTP server"
//     },
//     {
//         .at_cmd = "AT+CIPSNTPTIME?\r\n",
//         .expected_resp = "+CIPSNTPTIME:",
//         .timeout_ms = 3000,
//         .retries = 1,
//         .description = "Query network time"
//     },
    
//     // --------------- Error Recovery ---------------
//     {
//         .at_cmd = "AT+CWQAP\r\n",
//         .expected_resp = "OK",
//         .timeout_ms = 5000,
//         .retries = 2,
//         .description = "Disconnect from AP"
//     },
//     {
//         .at_cmd = "AT+RST\r\n",
//         .expected_resp = "ready",
//         .timeout_ms = 10000,
//         .retries = 1,
//         .description = "Module reboot"
//     },
    
//     // Termination marker
//     { NULL, NULL, 0, 0, NULL }
// };

