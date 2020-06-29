アームを取り付けて簡単に周波数応答を調べる

int A = 0.1*1024;
double freq = 2; // [Hz]
double omega = 2*3.14*freq;

int pos = 0;
int vel = 0;
int kp = 100;
int kd = 100;
int ff = 0;