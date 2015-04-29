//#include <Arduino.h>
//
//#define MAX 75
//#define MIN 0
//
//#define PP1_ 1
//#define PP2_ 0.3
//#define II1_ 1
//#define II2_ 1
//#define DD1_ 0.01
//#define DD2_ 0
//
//int motor1Pin = 5, motor2Pin = 6, motor3Pin = 9, motor4Pin = 10;
//
//int s1, s2;
//unsigned long s1t, s2t;
//
//double Setpoint1, Input1, Output1;
//double Setpoint2, Input2, Output2;
//double Setpoint3, Input3, Output3;
//double Setpoint4, Input4, Output4;
//
//double P1, I1, D1;
//double P2, I2, D2;
//double P3, I3, D3;
//double P4, I4, D4;
//
//unsigned long cuenta1, cuenta2, cuenta3, cuenta4;
//unsigned long m1, m2, m3, m4;
//
//int byteVelocidad1, byteComando1;
//int byteVelocidad2, byteComando2;
//int byteVelocidad3, byteComando3;
//int byteVelocidad4, byteComando4;
//
//int error1, Salida1;
//int error2, Salida2;
//int error3, Salida3;
//int error4, Salida4;
//
//boolean ejecutando = false;
//
//boolean direccion1 = false, direccion2 = false;
//boolean direccion3 = false, direccion4 = false;
//
//void sensores()
//{
////	s1 = digitalRead(A0);
////	s2 = digitalRead(A1);
////
////	if( s1 == LOW )
////	{
////		while( s1 == LOW )
////		{
////			s1 = digitalRead(A0);
////		}
////	}
////
////	while( s1 == HIGH )
////	{
////		s1 = digitalRead(A0);
////	}
////
////	s1t = millis();
////
////	while( s2 == LOW )
////	{
////		s2 = digitalRead(A1);
////	}
////
////	s2t = millis();
////
////	s1t = s2t-s1t;
////	s1 = digitalRead(A0);
////
////	while( s1 == LOW )
////	{
////		s1 = digitalRead(A0);
////	}
////
////	s2t = millis()-s2t;
////
////	m2 = s1t;
////	m1 = s2t;
//}
//
//void suma3()
//{
//	cuenta3 += 1;
//}
//
//void suma4()
//{
//	cuenta4 += 1;
//}
//
//void vel()
//{
//	m3 = cuenta3;
//	cuenta3 = 0;
//
//	m4 = cuenta4;
//	cuenta4 = 0;
//
//	Input1 = map(m1, MIN, MAX, 0, 255);
//	Input2 = map(m2, MIN, MAX, 0, 255);
//	Input3 = map(m3, MIN, MAX, 0, 255);
//	Input4 = map(m4, MIN, MAX, 0, 255);
//}
//
//template <typename type>
//type sign(type value) {
//	return type((value>0)-(value<0));
//}
