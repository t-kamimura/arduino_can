RMD-X8 PROの周波数応答確認

int A = 65536*0.3;
double f = 0.2~12.8; //[Hz]
double omega = 2*3.14*f;

int offset = 65536*0.5;  //A < offset