//============================================================================
// Name        : Server.cpp
// Author      : Jan Matejka
// Version     :
// Copyright   : Your copyright notice
// Description : PSI - Server application in C++, Ansi-style
//============================================================================

#include <iostream>
using namespace std;

#include <cstdlib>
#include <cstdio>
#include <sys/socket.h> // socket(), bind(), connect(), listen(), shutdown()
#include <unistd.h> // close(), read(), write()
#include <netinet/in.h> // struct sockaddr_in
#include <arpa/inet.h> // htons(), htonl()
#include <strings.h> // bzero()
#include <string>
#include <cstring>
#include <cmath>
#include <queue>

#define BUFFER_SIZE 100
#define TIMEOUT 1
#define TIMEOUT_RECHARGING 5

int sock = 0;
int c = 0;
char buffer[BUFFER_SIZE+2];

static const char* server_message[] = {
        "100 LOGIN\r\n",
        "101 PASSWORD\r\n",
        "102 MOVE\r\n",
        "103 TURN LEFT\r\n",
        "104 TURN RIGHT\r\n",
        "105 GET MESSAGE\r\n",
        "200 OK\r\n",
        "300 LOGIN FAILED\r\n",
        "301 SYNTAX ERROR\r\n",
        "302 LOGIC ERROR\r\n"
};

class CRobot{
private:
    int m_x;
    int m_x2;
    int m_x3;
    int m_y;
    int m_y2;
    int m_y3;
    int m_pass;	// hodnota hesla
    int m_len_pass; // delka vyuzivana pri pocitani hesla
    int m_conn; // cislo acceptu, na kterem nasloucha
    int m_cnt; // uzivatelske jmeno prevedene na cislo
    int m_phase; // zaznamenava fazi, ve ktere se robot nachazi; zjednodusuje orientaci v moznostech, ktere robot v techto fazich muze delat
    int m_phase_nav; // zaznamenava fazi, ve ktere se robot nachazi pri navadeni, tvori dvojici s m_quadrant
    int m_charging; // detekce, zda se robot prave nabiji
    int m_situation; // detekce, ke ktere situaci vstupu doslo
    bool m_flag_r; // detekce zda byl posledni znak \r
    int m_new_start; // detekce, zda doslo k prvni situaci, tedy nesmim zacit od zacatku bufferu
    char m_command[BUFFER_SIZE+1]; // retezec pro naplnovani jednotlivych prikazu
    int m_quadrant; // umisteni v 2D rovine
    int m_direction; // informace o natoceni robota
    int m_length; // delka retezce command
    int m_phase2; // pomocna faze v QueueWork()
public:
    CRobot(int c);
    //--- operace ----
    bool CommunicateAble();
    bool IsFinish();
    bool Receive();
    bool Running();
    void QueueWork(int length);
    bool HasAlreadyMoved(int mode); // mode slouzi k odliseni jestli porovnavam x a y nebo x2 a y2
    bool Navigate();
    void SetQuadrant();
    void SetDirection();
    //--- server -----
    bool ServerUser();
    bool ServerPass();
    bool ServerMove();
    bool ServerTurnLeft();
    bool ServerTurnRight();
    bool ServerPickUp();
    bool ServerOk();
    bool ServerLoginFailed();
    bool ServerSyntaxError();
    bool ServerLogicError();
    //----- client ----
    bool ClientUser();
    bool ClientPass();
    bool ClientConfirm();
    bool ClientMessage(char* buffer, int start);
};

void die(const string& str);
void dieAndClose(const string& str);


int main(int argc, char **argv) {
    //osetreni poctu argumentu
    if(argc < 2){
        cerr << "Usage: server port" << endl;
        return -1;
    }

    // Vytvoreni koncoveho bodu spojeni
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if(sock < 0){
        cerr << "Nemohu vytvorit socket!" << endl;
        return -1;
    }

    //konverze cisla portu
    int port = atoi(argv[1]);
    if(port == 0){
        die("Usage: server port");
    }

    //nastaveni ip adresy serveru
    struct sockaddr_in adresa;
    bzero(&adresa, sizeof(adresa));
    adresa.sin_family = AF_INET;
    adresa.sin_port = htons(port);
    adresa.sin_addr.s_addr = htonl(INADDR_ANY);

    // Prirazeni socketu k rozhranim
    if(bind(sock, (struct sockaddr *) &adresa, sizeof(adresa)) < 0){
        die("Problem s bind()!");
    }

    // Oznacim socket jako pasivni, takze bude aktivne naslouchat
    if(listen(sock, 10) < 0){
        die("Problem s listen()!");
    }

/*------------------------ konfigurace ke klientovi ------------------------- */
    //nastaveni ip adresy klienta
    struct sockaddr_in vzdalena_adresa;
    socklen_t velikost;

    while(true){
        // Cekam na prichozi spojeni
        //cout << "Jeste neprobehl accept " << i << endl;
        int c = accept(sock, (struct sockaddr *) &vzdalena_adresa, &velikost);
        //cout << "c = " << c <<endl;
        if(c < 0){
            die("Problem s accept()!");
        } else {
            //zalozim novy podproces
            pid_t pid = fork();
            //cout << "Jsem ve forku " << endl;
            if(pid == 0){
                //vytvoreni nove instance robot pro realizaci komunikace s klientem
                CRobot robot(c);
                //osetreni zda robot komunikuje spravne
                if(!robot.CommunicateAble()){
                    close(c);
                    shutdown(c,2);
                    return -2;
                }
                //pokud robot komunikoval spravne a uspesne ukoncil spojeni
                close(c);
                shutdown(c,2);
                return 0;
            }
            close(c);
        }
    }

    close(sock);
    return 0;
}

CRobot::CRobot(int c) : m_conn(c){
    m_x = 0;
    m_x2 = 0;
    m_x3 = 0;
    m_y = 0;
    m_y2 = 0;
    m_y3 = 0;
    m_cnt = 0;
    m_pass = 0;
    m_len_pass = 0;
    m_phase = 0;
    m_phase_nav = 0;
    m_charging = 0;
    m_situation = 0;
    m_flag_r = false;
    m_new_start = 0;
    m_quadrant = 0;
    m_direction = 0;
    m_command[0] = 0;
    m_length = 0;
    m_phase2 = 0;
}

bool CRobot::CommunicateAble(){
    //pozadavek na uzivatelske jmeno
    if(!ServerUser())
        return 0;
    if(!Receive())
        return 0;
    return 1;
}

// odesila klientovi zpravu
bool CRobot::ServerUser(){
    if(send(m_conn, server_message[0], strlen(server_message[0]), 0) >= 0)
        return true;
    else
        return false;
}

// validuje zda uzivatelske jmeno nema nulovou delku
bool CRobot::ClientUser(){
    //cout << "m_command: " << m_command << endl;
    for(int i = 0; (int)m_command[i] != 13 || (int)m_command[i+1] != 10; i++){
        if(i >= BUFFER_SIZE){
            //cout << "ClientUser i preteklo buffer." << endl;
            return false;
        }
        //cout << "m_command[i] = " << (int)m_command[i] << ", i = " << i << endl;
        m_cnt += (int)m_command[i];
    }
    if(m_cnt == 0)
        return false;
    return true;
}

// odesila zpravu klientovi
bool CRobot::ServerPass(){
    if(send(m_conn, server_message[1], strlen(server_message[1]), 0) >= 0)
        return true;
    else
        return false;
}

// porovnava uzivatelske jmeno s heslem
bool CRobot::ClientPass(){
    // jeste validace, zda nema rezezec vic nez 7 znaku vcetne \r\n
    // cyklus na projduti atd. !!!
    //cout << "client pass: " << m_command << endl;

    if(m_length > 7){
        cout << "Heslo obsahuje vice nez 7 znaku." << endl;
        ServerSyntaxError();
        return false;
    }
    int i = 0;
    while((int)m_command[i] != 13){
        if((int)m_command[i]-48 < 0 || (int)m_command[i]-48 > 9){
            cout << "Heslo obsahuje i neciselne hodnoty." << endl;
            ServerSyntaxError();
            return false;
        }
        i++;
    }
    m_len_pass = i-1;
    //cout << "m_len_pass: " << m_len_pass << endl;
    for(i = 0; i <= m_len_pass; i++){
        m_pass += (m_command[i]-48)*pow(10,(m_len_pass)-i);
        //cout << "m_command[i]: " << m_command[i] << endl;
        //cout << "hodnota hesla: " << m_pass << endl;
    }
    if(m_cnt == m_pass)
        return true;
    else {
        if(!ServerLoginFailed())
            return false;
        return false;
    }
}

// odesila zpravu klientovi
bool CRobot::ServerOk(){
    if(send(m_conn, server_message[6], strlen(server_message[6]), 0) >= 0)
        return true;
    else
        return false;
}

// odesila zpravu klientovi
bool CRobot::ServerLoginFailed(){
    if(send(m_conn, server_message[7], strlen(server_message[7]), 0) >= 0)
        return true;
    else
        return false;
}

bool CRobot::ClientConfirm(){
    //char* buffer2 = &m_command[0];
    char input[2], r;
    float x,y;
    //cout << "buffer2[0] = " << buffer2[0] << ", buffer2 = " << buffer2 << endl;
    if(sscanf(m_command, "%s%f%f%c", input, &x, &y, &r) < 4 || strcmp(input,"OK")  || r != 13){
        cout << "ClientConfirm: " << input << " " << x << " " << y << endl;
        cout << "Chybne prijmute souradnice." << endl;
        return false;
    }
    //cout << "ClientConfirm: " << input << " " << x << " " << y << endl;
    if((x-(int)x) > 0.0)
        return false;
    if((y-(int)y) > 0.0)
        return false;
    /*
    cout << "x = " << x << ", y = " << y << endl;
    cout << "m_x = " << m_x << ", m_y = " << m_y << endl;
    cout << "m_x2 = " << m_x2 << ", m_y2 = " << m_y2 << endl;
    cout << "---------------------------------------" << endl;
    */
    // jestlize jsem uz provedl alespon jeden SERVER_MOVE
    if(HasAlreadyMoved(1)){
        m_x2 = m_x;
        m_y2 = m_y;
        m_x = x;
        m_y = y;
        // urcite nutne na zacatku
        if(m_direction == 0)
            SetDirection();

        if(m_direction == 0){
            m_x2 = 0;
            m_y2 = 0;
        }
        // jestlize jsem zatim neprovedl zadny SERVER_MOVE
    }else{
        m_x = x;
        m_y = y;
        /*
        cout << "x = " << x << ", y = " << y << endl;
        cout << "m_x = " << m_x << ", m_y = " << m_y << endl;
        cout << "m_x2 = " << m_x2 << ", m_y2 = " << m_y2 << endl;
        cout << "---------------------------------------" << endl;
        */
        SetQuadrant();
    }

    // zatim si nejsem jisty
    /*if(HasAlreadyMoved(2))
        SetDirection();
    */

    //cout << "x = " << m_x << ", y = " << m_y << ", input = " << input << endl;
    return true;
}

bool CRobot::IsFinish(){
    if (m_x == 0 && m_y == 0)
        return true;
    return false;
}

// odesila zpravu klientovi
bool CRobot::ServerPickUp(){
    if(send(m_conn, server_message[5], strlen(server_message[5]), 0) >= 0)
        return true;
    else
        return false;
}

// odesila zpravu klientovi
bool CRobot::ServerMove(){
    if(send(m_conn, server_message[2], strlen(server_message[2]), 0) >= 0)
        return true;
    else
        return false;
}

bool CRobot::ClientMessage(char* buffer, int start){
    int l = start;
    while(buffer[l] != 13)
        l++;
    if(start == l)
        return false;
    //cout <<  << endl;
    return true;
}

// odesila zpravu klientovi
bool CRobot::ServerTurnLeft(){
    if(send(m_conn, server_message[3], strlen(server_message[3]), 0) >= 0)
        return true;
    else
        return false;
}

// odesila zpravu klientovi
bool CRobot::ServerTurnRight(){
    if(send(m_conn, server_message[4], strlen(server_message[4]), 0) >= 0)
        return true;
    else
        return false;
}

// odesila zpravu klientovi
bool CRobot::ServerSyntaxError(){
    if(send(m_conn, server_message[8], strlen(server_message[8]), 0) >= 0)
        return true;
    else
        return false;
}

// odesila zpravu klientovi
bool CRobot::ServerLogicError(){
    if(send(m_conn, server_message[9], strlen(server_message[9]), 0) >= 0)
        return true;
    else
        return false;
}

bool CRobot::Running(){
    /*cout << "m_phase = " << m_phase << endl;
    cout << "m_command v Running = " << m_command << endl;
    cout << "'";
    for(int i = 0; m_command[i] != 0; i++){
        cout << (int)m_command[i] << " ";
    }
    cout << "'" << endl;*/
    //cout << "m_pass: " << m_pass << ", m_cnt: " << m_cnt << endl;

    /*cout << "m_command.length() = " << m_command.length() << endl;
    cout << "predposledni: " << m_command[m_command.length()-2] << ", posledni =: " << m_command[m_command.length()-1] << endl;

    if(m_command[m_command.length()-2] != 13 && m_command[m_command.length()-1] != 10){
        ServerSyntaxError();
        return false;
    }*/

    //detekce, zda se robot dobil a spravne odpovedel
    if(m_charging){
        //cout << "m_command v Running = " << m_command << endl;
        if(!strncmp(m_command, "FULL POWER\r\n", 12)){
            m_charging = 0;
        }else{
            ServerLogicError();
            cout << "Retezec se nerovna FULL POWER." << endl;
            return false;
        }

        // pokud jsem detekoval pozadavek o nabijeni, tak nastavim priznak a cekam delsi dobu na full power
    }else if(!strncmp(m_command, "RECHARGING\r\n", 12)){
        // nastavim priznak, ze se robot nabiji
        m_charging = 1;
        //cout << "Nabijim" << endl;
    }else{
        //cout << "m_phase = " << m_phase << endl;
        switch(m_phase){
            case 0:
                if(ClientUser()){
                    if(!ServerPass())
                        return false;
                    m_phase++;
                }else{
                    if(!ServerSyntaxError())
                        return false;
                    return false;
                }
                break;
            case 1:
                if(ClientPass()){
                    if(!ServerOk())
                        return false;
                    if(!ServerMove())
                        return false;
                    m_phase++;
                }else{
                    return false;
                }
                break;
            case 2:
                if(m_situation == 2){
                    if(ClientConfirm()){
                        if(IsFinish()){
                            if(!ServerPickUp()){
                                return false;
                            }
                            m_phase++;
                            //return true;
                        }else{
                            // nejsem v cili, takze se tam musim navest
                            if(!Navigate()){
                                // error
                                return false;
                            }

                        }
                    }else{
                        if(!ServerSyntaxError())
                            return false;
                        return false;
                    }
                }else{
                    if(ClientConfirm()){
                        if(IsFinish()){
                            if(!ServerPickUp()){
                                return false;
                            }
                            m_phase++;
                            //return true;
                        }else{
                            // nejsem v cili, takze se tam musim navest
                            if(!Navigate()){
                                // error
                                return false;
                            }

                        }
                    }else{
                        if(!ServerSyntaxError())
                            return false;
                        return false;
                    }
                }
                break;
            case 3:
                if(m_situation == 2){
                    if(!ClientMessage(m_command, m_new_start)){
                        ServerSyntaxError();
                        return false;
                    }
                    if(!ServerOk())
                        return false;
                    m_phase++;
                    //return true;
                }else{
                    ServerSyntaxError();
                    return false;
                }

                break;
        }
    }
    return true;
}

bool CRobot::Receive(){
    fd_set sockets;
    FD_ZERO(&sockets);
    FD_SET(m_conn, &sockets);

    int bytesRead = 0;
    m_situation = 0;
    int i = 0; // iterator prijimaneho retezce
    while(true){
        if(m_situation != 1 && m_situation != 4){
            // nastaveni timeoutu
            struct timeval timeout;
            timeout.tv_sec = (m_charging) ? TIMEOUT_RECHARGING : TIMEOUT;
            timeout.tv_usec = 0;

            int retval = select(m_conn + 1, &sockets, NULL, NULL, &timeout);
            if(retval < 0){
                return false;
            }
            if(!FD_ISSET(m_conn, &sockets)){
                // Zde je jasne, ze funkce select() skoncila cekani kvuli timeoutu.
                cout << "Connection timeout!" << endl;
                return false;
            }
            bytesRead = recv(m_conn, buffer, BUFFER_SIZE+2, 0);
            if(bytesRead <= 0)
                return false;
            //cout << "bytesRead = " << bytesRead << endl;
            if(bytesRead > BUFFER_SIZE){
                if(!ServerSyntaxError())
                    return false;
                return false;
            }
            buffer[bytesRead] = 0;
            //obnovim, jelikoz pouze v 1.situaci nezacinam od zacatku retezce
            i = 0;
        }


        m_new_start = 0;
        //pokud nastala prvni situace, mel bych nastavit novy zacatek
        if(m_situation == 1 && !m_flag_r){
            i+=2;
            m_new_start = i; // musim zacit az za \r\n
        }else if (m_situation == 1 && m_flag_r){
            m_flag_r = false;
            i++;
            m_new_start = i;
        }

        if(m_situation == 4){
            i++;
            m_new_start = i;
        }

        char * p_buf = &buffer[m_new_start];
        int recharge = 0;
        if(!strcmp(p_buf, "RECHARGING\r\n") || !strcmp(p_buf, "FULL POWER\r\n")){
            recharge = 1;
        }

        // pocet mezer - vhodne pro validaci souradnic
        int count_of_space = 0;


        //dokud nenaleznu znak 13 a nedostal jsem se na posledni znak prijmuteho retezce
        while((int)buffer[i] != 13 && i < bytesRead){
            //cout << "m_len_pass: " << m_len_pass << endl;
            if(m_flag_r && i == 0 && (int)buffer[i] == 10){
                break;
            }

            /*if(m_phase == 0){
                //cout << "buffer[i] = " << (int)buffer[i] << ", i = " << i << endl;
                m_cnt += (int)buffer[i];
            }*/

            // odpoved se souradnicemi muze obsahovat pouze 2 mezery, tedy je nutne to osetrit
            if(m_phase == 2 && !recharge){
                if((int)buffer[i] == 32)
                    count_of_space++;
                if(count_of_space > 2){
                    ServerSyntaxError();
                    return false;
                }

            }
            i++;
        }

        // nastavovani ukonceni retezce
        // jestlize jsem se dostal za buffer neboli, ze nastala 3. situace
        if(i == bytesRead){
            buffer[i] = 0;
        }
            // jestlize je posledni znak \r
        else if((i+1) == bytesRead && ((int)buffer[i] == 13 || m_flag_r)){
            buffer[i+1] = 0;
        }
            // jestlize je predposledni znak \r a posledni \n
        else if((i+2) == bytesRead && (int)buffer[i] == 13 && (int)buffer[i+1] == 10){
            buffer[i+2] = 0;
        }

        /*// pokud nacitam uzivatelske jmeno a nacetl jsem \r bez \n, tak pripoctu k hodnote hesla
        if(m_phase == 0){
            if((int)buffer[i] == 13 && (int)buffer[i+1] != 10){
                //cout << "buffer[i] = " << (int)buffer[i] << ", i = " << i << endl;
                m_cnt += (int)buffer[i];
            }
        }*/

        // validace zda neni nacteny retezec delsi nez BUFFER
        if(i+1 > BUFFER_SIZE || (bytesRead >= BUFFER_SIZE && (int)buffer[bytesRead-1] != 10)){
            //cout << "i = " << i << ", bytesRead: " << bytesRead << ", situation: " << m_situation << endl;
            ServerSyntaxError();
            return false;
        }

        int len = 0;
        // ukoncovani retezce
        // jestlize je na predposlednim znaku /r a poslednim /n
        if((int)buffer[i] == 13 && (int)buffer[i+1] == 10){

            if(m_situation == 4){
                len = i - m_new_start + 2;
            }else{
                // pro situaci 0,1,2 a 3 je to v poradku
                len = i - m_new_start + 2;
            }

            // jestlize je na predposlednim znaku /r, ale na poslednim neni /n
        }else if((int)buffer[i] == 13 && (int)buffer[i+1] != 10){

            len = i - m_new_start + 1;

            // jestlize jsem v predeslem retezci nalezl \r a nyni na prvni pozici \n
        }else if(m_flag_r && i == 0 && (int)buffer[i] == 10){

            len = i + 1;

            // jestlize na predposlednim znaku neni /r a za nim je cokoliv - tj. dostali jsme se na konec bufferu bez nalezeni /r/n
        }else{

            if(m_situation == 1){
                len = i - m_new_start;
                // nejspis tady nastava pouze situace 3
            }else{
                len = i;
            }
        }
        //cout << "m_command = " << m_command << endl;
        //cout << "m_cnt = " << m_cnt << ", m_pass = " << m_pass << endl;


        /* 1.situace, pokud neni posledni znak na prijmutem retezci \n a retezec pokracuje */
        if(((int)buffer[i+1] == 10 && (i+2) < bytesRead) || (m_flag_r && (int)buffer[i] == 10 && (i+1) < bytesRead)){
            // prijmul jsem cely prikaz, takze zmenim priznak
            //m_flag_r = false;
            //pokud uz jsem jiz nacetl nedokonceny retezec

            // spustim vyprazdneni fronty do jednoho stringu prikazu
            QueueWork(len);

            // nastavim flag, ze nastala situace 1
            m_situation = 1;
            // zavolam metodu prubehu
            if(!Running())
                return false;

            // vynuluji retezec prikazu
            m_command[0] = 0;
            m_length = 0;

            // pokracuji v iteraci
            continue;
        }
            /* 2.situace, pokud je posledni znak na prijmutem retezci \n a retezec konci */
        else if(((int)buffer[i+1] == 10 && (i+2) == bytesRead) || (m_flag_r && (int)buffer[i] == 10 && (i+1) == bytesRead)){
            //cout << "Nazdar" << endl;
            // prijmul jsem cely prikaz, takze zmenim priznak
            m_flag_r = false;
            //pokud uz jsem jiz nacetl nedokonceny retezec

            // spustim vyprazdneni fronty do jednoho stringu prikazu
            QueueWork(len);

            //cout << "Problem ?" << endl;

            // nastavim flag, ze nastala situace 2
            m_situation = 2;
            // zavolam metodu prubehu
            if(!Running())
                return false;

            // vynuluji retezec prikazu
            m_command[0] = 0;
            m_length = 0;

            // prijmul jsem zpravu a ukoncuji spojeni
            if(m_phase == 4){
                break;
            }
            // pokracuji v iteraci
            continue;
            /* 4. situace, pokud jsem prijmul jen \r a cokoli za nim krome \n */
        }else if((int)buffer[i] == 13 && (int)buffer[i+1] != 10 && (i+1) < bytesRead){
            // validace pokud je delka prikazu nulova
            /*if(!m_command.length()){
                ServerSyntaxError();
                return false;
            }*/
            //cout << "m_new_start = " << m_new_start << endl;
            //cout << "i = " << i << endl;
            //cout << "len = " << len << endl;
            // vloz na konec fronty prikazu
            QueueWork(len);
            // nastav flag, ze nastala situace 4
            m_situation = 4;
            continue;
            /* 5.situace, pokud jsem prijmul jen \r na konci retezce */
        }else if((int)buffer[i] == 13 && (int)buffer[i+1] != 10 && (i+1) == bytesRead){
            // nastavim priznak, ze jsem prijmul \r, ale zatim zadne \n, takze jej ocekavam
            m_flag_r = true;

            // vloz na konec fronty prikazu
            QueueWork(len);

            m_situation = 3;
            continue;
            /* 3.situace, pokud jsem neprijmul \r\n */
        }else{
            // validace pokud je delka prikazu nulova
            if(!len){
                ServerSyntaxError();
                return false;
            }

            /*// nastavim priznak, ze jsem prijmul \r, ale zatim zadne \n, takze jej ocekavam
            if((int)buffer[i] == 13){
                m_flag_r = true;
            }*/
            // vloz na konec fronty prikazu
            QueueWork(len);
            // nastav flag, ze nastala situace 3
            m_situation = 3;
            continue;
        }
    }

    return true;
}

// prace fronty s neukoncenymi retezci
void CRobot::QueueWork(int length){
    /*cout << "m_situation: " << m_situation << endl;
    cout << "-------- zacatek QueueWork ----------------" << endl;
    cout << "buffer: " << endl;
    for(int i = 0; i != 20; i++){
        cout << buffer[i] << " = " << (int)buffer[i] << endl;
    }*/

    // rozdeleni chovani podle situaci
    if(m_situation == 0 || m_situation == 2){
        // nastavim delku retezce
        m_length = length;
        // zkopiruji retezec delky length
        strncpy(m_command, buffer, length);
    }else if(m_situation == 1 || m_situation == 4){
        if(!m_length){
            // nastavim ukazatel na novy zacatek
            char* p_buf = &buffer[m_new_start];
            // nastavim delku retezce
            m_length = length;
            // zkopiruji retezec delky length
            strncpy(m_command, p_buf, length);
        }else{
            // nastavim ukazatel na novy zacatek
            /*char* p_buf = &buffer[m_new_start];
            char* p_buf2 = &m_command[m_length];
            // nastavim delku retezce
            m_length += length;
            // zkopiruji retezec delky length
            strncat(p_buf2, p_buf, length);*/
            for(; length; m_new_start++,m_length++,--length){
                m_command[m_new_start] = buffer[m_length];
            }
            //m_command[m_length++] = 0;

        }
    }else if(m_situation == 3){
        if(!m_length){
            // nastavim delku retezce
            m_length = length;
            // zkopiruji retezec delky length
            strncpy(m_command, buffer, length);
        }else{
            // nastavim ukazatel na novy zacatek
            /*char* p_buf = &m_command[m_length];
            cout << "sit: m_length: " << m_length << endl;
            // prictu novou delku k te stavajici
            m_length += length;
            // pripojim ke stavajicimu retezci novy retezec
            strncat(m_command, buffer, length);*/

            for(;length;m_length++,m_new_start++,--length){
                m_command[m_length] = buffer[m_new_start];
            }
        }
    }
    /*cout << "m_length: " << m_length << endl;
    cout << "length: " << length << endl;
    cout << "-------------------------------------------------------" << endl;
    cout << "m_command: " << endl;
    for(int i = 0; i != 20; i++){
        cout << m_command[i] << " = " << (int)m_command[i] << endl;
    }
    cout << "------------------- konec QueueWork ------------------" << endl;
    cout << endl << endl;

    //cout << "m_command = " << m_command << endl;*/
}


bool CRobot::HasAlreadyMoved(int mode){
    if(mode == 1){
        if(m_x == 0 && m_y == 0)
            return false;
        return true;
    }else if(mode == 2){
        if(m_x2 == 0 && m_y2 == 0)
            return false;
        return true;
    }
    return false;
}

bool CRobot::Navigate(){
    switch(m_quadrant){
        // 1.kvadrant
        case 1:
            switch(m_direction){
                case 1:
                    if(!ServerTurnRight())
                        return false;
                    m_direction = 2;
                    break;
                case 2:
                    if(m_x != 0){
                        if(!ServerMove())
                            return false;
                        m_direction = 2;
                    }else{
                        if(!ServerTurnRight())
                            return false;
                        m_quadrant = 6;
                        m_direction = 3;
                    }
                    break;
                case 3:
                    if(m_y != 0){
                        if(!ServerMove())
                            return false;
                        m_direction = 3;
                    }else{
                        if(!ServerTurnLeft())
                            return false;
                        m_quadrant = 5;
                        m_direction = 2;
                    }
                    break;
                case 4:
                    if(!ServerTurnLeft())
                        return false;
                    m_direction = 3;
                    break;
                default:
                    if(!ServerMove())
                        return false;
                    break;
            }
            break;
            // 2.kvadrant
        case 2:
            switch(m_direction){
                case 1:
                    if(!ServerTurnLeft())
                        return false;
                    m_direction = 4;
                    break;
                case 2:
                    if(!ServerTurnRight())
                        return false;
                    m_direction = 3;
                    break;
                case 3:
                    if(m_y != 0){
                        if(!ServerMove())
                            return false;
                        m_direction = 3;
                    }else{
                        if(!ServerTurnRight())
                            return false;
                        m_quadrant = 7;
                        m_direction = 4;
                    }
                    break;
                case 4:
                    if(m_x != 0){
                        if(!ServerMove())
                            return false;
                        m_direction = 4;
                    }else{
                        if(!ServerTurnLeft())
                            return false;
                        m_quadrant = 6;
                        m_direction = 3;
                    }
                    break;
                default:
                    if(!ServerMove())
                        return false;
                    break;
            }
            break;
            // 3.kvadrant
        case 3:
            switch(m_direction){
                case 1:
                    if(m_y != 0){
                        if(!ServerMove())
                            return false;
                        m_direction = 1;
                    }else{
                        if(!ServerTurnRight())
                            return false;
                        m_quadrant = 5;
                        m_direction = 2;
                    }
                    break;
                case 2:
                    if(m_x != 0){
                        if(!ServerMove())
                            return false;
                        m_direction = 2;
                    }else{
                        if(!ServerTurnLeft())
                            return false;
                        m_quadrant = 8;
                        m_direction = 1;
                    }
                    break;
                case 3:
                    if(!ServerTurnLeft())
                        return false;
                    m_direction = 2;
                    break;
                case 4:
                    if(!ServerTurnRight())
                        return false;
                    m_direction = 1;
                    break;
                default:
                    if(!ServerMove())
                        return false;
                    break;
            }
            break;
            // 4.kvadrant
        case 4:
            switch(m_direction){
                case 1:
                    if(m_y != 0){
                        if(!ServerMove())
                            return false;
                        m_direction = 1;
                    }else{
                        if(!ServerTurnLeft())
                            return false;
                        m_quadrant = 7;
                        m_direction = 4;
                    }
                    break;
                case 2:
                    if(!ServerTurnLeft())
                        return false;
                    m_direction = 1;
                    break;
                case 3:
                    if(!ServerTurnRight())
                        return false;
                    m_direction = 4;
                    break;
                case 4:
                    if(m_x != 0){
                        if(!ServerMove())
                            return false;
                        m_direction = 4;
                    }else{
                        if(!ServerTurnRight())
                            return false;
                        m_quadrant = 8;
                        m_direction = 1;
                    }
                    break;
                default:
                    if(!ServerMove())
                        return false;
                    break;
            }
            break;
            // 5.kvadrant - zaporna osa x
        case 5:
            switch(m_direction){
                case 1:
                    if(!ServerTurnRight())
                        return false;
                    m_direction = 2;
                    break;
                case 2:
                    if(m_x != 0){
                        if(!ServerMove())
                            return false;
                        m_direction = 2;
                    }
                    break;
                case 3:
                    if(!ServerTurnLeft())
                        return false;
                    m_direction = 2;
                    break;
                case 4:
                    if(!ServerTurnLeft())
                        return false;
                    m_direction = 3;
                    break;
                default:
                    if(!ServerMove())
                        return false;
                    break;
            }
            break;
            // 6.kvadrant - kladna osa y
        case 6:
            switch(m_direction){
                case 1:
                    if(!ServerTurnRight())
                        return false;
                    m_direction = 2;
                    break;
                case 2:
                    if(!ServerTurnRight())
                        return false;
                    m_direction = 3;
                    break;
                case 3:
                    if(m_y != 0){
                        if(!ServerMove())
                            return false;
                        m_direction = 3;
                    }
                    break;
                case 4:
                    if(!ServerTurnLeft())
                        return false;
                    m_direction = 3;
                    break;
                default:
                    if(!ServerMove())
                        return false;
                    break;
            }
            break;
            // 7.kvadrant - kladna osa x
        case 7:
            switch(m_direction){
                case 1:
                    if(!ServerTurnLeft())
                        return false;
                    m_direction = 4;
                    break;
                case 2:
                    if(!ServerTurnLeft())
                        return false;
                    m_direction = 1;
                    break;
                case 3:
                    if(!ServerTurnRight())
                        return false;
                    m_direction = 4;
                    break;
                case 4:
                    if(m_x != 0){
                        if(!ServerMove())
                            return false;
                        m_direction = 4;
                    }
                    break;
                default:
                    if(!ServerMove())
                        return false;
                    break;
            }
            break;
            // 8.kvadrant - zaporna osa y
        case 8:
            switch(m_direction){
                case 1:
                    if(m_y != 0){
                        if(!ServerMove())
                            return false;
                        m_direction = 1;
                    }
                    break;
                case 2:
                    if(!ServerTurnLeft())
                        return false;
                    m_direction = 1;
                    break;
                case 3:
                    if(!ServerTurnLeft())
                        return false;
                    m_direction = 2;
                    break;
                case 4:
                    if(!ServerTurnRight())
                        return false;
                    m_direction = 1;
                    break;
                default:
                    if(!ServerMove())
                        return false;
                    break;
            }
            break;
        default:
            if(!ServerMove())
                return false;
            break;
    }
    return true;
}

// nastavuji umisteni v 2D rovine
void CRobot::SetQuadrant(){
    if(m_x < 0 && m_y > 0)
        m_quadrant = 1;
    else if(m_x > 0 && m_y > 0)
        m_quadrant = 2;
    else if(m_x < 0 && m_y < 0)
        m_quadrant = 3;
    else if(m_x > 0 && m_y < 0)
        m_quadrant = 4;
    else if(m_x < 0 && m_y == 0)
        m_quadrant = 5;
    else if(m_x == 0 && m_y > 0)
        m_quadrant = 6;
    else if(m_x > 0 && m_y == 0)
        m_quadrant = 7;
    else if(m_x == 0 && m_y < 0)
        m_quadrant = 8;
    else
        m_quadrant = 0;
}

// nastaveni smeru podle souradnic x,y a x2,y2
void CRobot::SetDirection(){
    if(m_x2 == m_x && m_y2 < m_y)
        m_direction = 1;
    else if(m_x2 < m_x && m_y2 == m_y)
        m_direction = 2;
    else if(m_x2 == m_x && m_y2 > m_y)
        m_direction = 3;
    else if(m_x2 > m_x && m_y2 == m_y)
        m_direction = 4;
    else
        m_direction = 0;
}


void die(const string& str){
    cerr << str << endl;
    close(sock);
    exit(-1);
}

void dieAndClose(const string& str){
    close(c);
    die(str);
}
