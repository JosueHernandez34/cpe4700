#ifndef ENCODER_H
#define ENCODER_H

int enc_init(void);
int enc_get_count(int encoder);
double enc_get_speed(int encoder);
int enc_cleanup(void);

#endif
