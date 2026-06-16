#ifndef PROTO_MOTOR_H
#define PROTO_MOTOR_H

int ProtoMotor_MakeStopCmd(char *buf, unsigned int buf_size);
int ProtoMotor_MakeSpeedCmd(char *buf, unsigned int buf_size, int m1, int m2, int m3, int m4);

#endif /* PROTO_MOTOR_H */
