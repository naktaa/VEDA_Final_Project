#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <openssl/md5.h>
#include <stdio.h>
#include <string.h>

// MD5 결과를 문자열로 변환하는 함수
void hash_to_str(unsigned char *hash, char *out) {
    for(int i = 0; i < 16; i++) {
        sprintf(out + (i * 2), "%02x", hash[i]);
    }
}

// Digest Response 계산 함수
void calculate_response(char *nonce, char *result) {
    char *user = "admin";
    char *pass = "5hanwha!";
    char *realm = "iPOLiS";
    char *method = "DESCRIBE";
    char *uri = "rtsp://192.168.100.3/0/profile2/media.smp";

    unsigned char hash[16];
    char ha1_str[33], ha2_str[33];
    char temp[512];

    // 1. HA1 계산
    sprintf(temp, "%s:%s:%s", user, realm, pass);
    MD5((unsigned char*)temp, strlen(temp), hash);
    hash_to_str(hash, ha1_str);

    // 2. HA2 계산
    sprintf(temp, "%s:%s", method, uri);
    MD5((unsigned char*)temp, strlen(temp), hash);
    hash_to_str(hash, ha2_str);

    // 3. 최종 Response 계산
    sprintf(temp, "%s:%s:%s", ha1_str, nonce, ha2_str);
    MD5((unsigned char*)temp, strlen(temp), hash);
    hash_to_str(hash, result);
}

int main() {
    int sock;
    struct sockaddr_in server;
    char message[1000], server_reply[2000];

    // 1. 소켓 생성 (TCP)
    sock = socket(AF_INET, SOCK_STREAM, 0);
    
    server.sin_addr.s_addr = inet_addr("192.168.100.3"); // 카메라 IP
    server.sin_family = AF_INET;
    server.sin_port = htons(554); // RTSP 기본 포트

    // 2. 카메라 접속
    if (connect(sock, (struct sockaddr *)&server, sizeof(server)) < 0) {
        puts("접속 실패!");
        return 1;
    }
    puts("카메라 554번 포트에 연결되었습니다.");

    // 3. RTSP DESCRIBE 메시지 작성 (이게 핵심!)
    // ID/PW가 있는 경우 실제로는 인증 헤더가 필요하지만, 우선 기본 틀이야.
    sprintf(message, 
        "DESCRIBE rtsp://192.168.100.3/0/profile2/media.smp RTSP/1.0\r\n"
        "CSeq: 1\r\n"
        "User-Agent: MyCustomPlayer\r\n"
        "Accept: application/sdp\r\n\r\n");

    // 4. 메시지 전송
    if (send(sock, message, strlen(message), 0) < 0) {
        puts("전송 실패");
        return 1;
    }

    // 5. 카메라의 응답 수신
    if (recv(sock, server_reply, 2000, 0) < 0) {
        puts("응답 받기 실패");
    }

    printf("1차 응답:\n%s\n", server_reply);

    // 6. nonce 추출
    char *nonce_ptr = strstr(server_reply, "nonce=\"");
    if (nonce_ptr) {
        nonce_ptr += 7; // "nonce=\"" 다음 지점으로 이동
        char nonce[100] = {0};
        char *end_ptr = strchr(nonce_ptr, '\"');
        strncpy(nonce, nonce_ptr, end_ptr - nonce_ptr);

        // --- 여기서 함수 호출! ---
        char response[33];
        calculate_response(nonce, response); 
        // -----------------------

        // 7. 인증 정보를 포함한 2차 DESCRIBE 메시지 작성
        char auth_message[1500];
        sprintf(auth_message,
            "DESCRIBE rtsp://192.168.100.3/0/profile2/media.smp RTSP/1.0\r\n"
            "CSeq: 2\r\n" // 시퀀스 번호 증가
            "Authorization: Digest username=\"admin\", realm=\"iPOLiS\", "
            "nonce=\"%s\", uri=\"rtsp://192.168.100.3/0/profile2/media.smp\", "
            "response=\"%s\"\r\n"
            "User-Agent: MyCustomPlayer\r\n"
            "Accept: application/sdp\r\n\r\n", 
            nonce, response);

        // 8. 2차 메시지 전송
        send(sock, auth_message, strlen(auth_message), 0);

        // 9. 최종 응답 수신 (200 OK 예상)
        memset(server_reply, 0, sizeof(server_reply));
        recv(sock, server_reply, 2000, 0);
        printf("2차 응답 (최종):\n%s\n", server_reply);
        }
    close(sock);
    return 0;
}