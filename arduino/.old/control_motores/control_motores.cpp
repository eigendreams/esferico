/*
 * El sistema de control descrito posteriormente es en realidad bastante simple. Un bucle de control
 * se ejecuta de forma constante emitiendo valores de salida PWM a una frecuencia fija de 50 Hz.
 * Dependiendo del valor de algunos de los tópicos usados en el programa, o bien estos valores se
 * asignan por medio de un bucle de control PID (también a 50 Hz), o bien se pasan DIRECTAMENTE (aunque
 * limitados a 0-255 por si acaso) a partir del tópico llamado vdes.
 *
 * La velocidad de toma de datos del encoder es la misma que la del bucle PID, lo que se puede
 * cambiar es la tasa de TRANSMISIÓN de datos del uC a ROS. Se puede especificar, de manera similar
 * al bucle PID, si se desea que se transmitan los datos procesados y filtrados de velocidad angular
 * de cada rueda, o solamente las lecturas de analogRead en cada encoder. El encoder siempre se
 * ejecuta, independientemente del modo de funcionamiento, pero no se realiza la parte de las
 * suma de datos registrados de variación, y ahorra un poco de procesamiento. Esto parece
 * excesivo, pero permite evitar el saturar los nodos de procesamiento en el lado de ROS por
 * una avalancha continua de datos.
 *
 * Sin embargo, el encoder SIEMPRE realiza el cálculo de la velocidad angular instantánea de TODOS
 * modos, pues puede ser que aún se requiera que los valores publicados de vdes especifiquen una
 * valocidad angular deseada, a pesar de que las lecturas del encoder se recolecten sin procesamiento
 * alguno. Por qué se querría esto no lo sé, pero es posible, por ejemplo, con los encoders analógicos
 * es posible ejercer así pruebas de funcionamiento más precisas.
 *
 * El resumen de los tópics es:
 *
 * ws, un Float32MultiArray que PUBLICA los datos de los encoders, o bien en forma de las lecturas
 *  de los encoders de 0 a 1023, o bien en forma de valores de velocidad angular en rads/s, según el
 *  modo que se defina en algunos otros tópicos. Las dos posibles formas de salida son entonces:
 *
 *  [lectura_derecha, timeStamp_derecha, lectura_izquierda, timeStamp_izquierda]
 *  [delta_phi_derecha, delta_t_derecha, delta_phi_izquierda, delta_t_izquierda]
 *
 *  Dependiendo del modo de salida del encoder usado
 *
 * vdes, un Float32MultiArray que RECIBE los datos de o bien velocidad deseada o bien de PWM que
 *  se desea en los motores, según el modo de entrada declarado. El primer valor para el motor derecho
 *  y el segundo valor para el motor izquierdo
 *
 * kpid, un Float32MultiArray que RECIBE los datos de las constantes PID y la constante del motor,
 *  en el orden: P, I, D, cte_motor. Si no se usa el bucle de control PID, es conveniente eliminar
 *  el código del topic para ahorrar RAM
 *
 * R0, un flotante que especifica el "radio" de valores permisibles de salida PWM DEL BUCLE PID,
 * 	alrededor de el valor de cero, de 127.5, esto es, si R0 fuera 100, el PID daria como
 * 	máximo 227.5 y como mínimo 27.5
 *
 * Mc, un booleando que especifica el "modo de control". Si True, los valores de vdes se asumen
 * 	como PWM y se pasan directo a las salidas (limitados de 0 a 255), si False, se asume que se
 * 	desea ejecutar el control por medio del bucle PID implementado en el uC. El bucle PID NO
 * 	se ejecuta en el primer modo, de modo que el procesamiento es más rápido y menos propenso a
 * 	retrasos, pero de todos modos se añade un retraso de comunicación con ROS si el control se
 * 	implementa fuera del uC que podría ser malo
 *
 * Ts, un entero sin signo que especifica el tiempo de publicación de los datos de los encoders.
 *  El tiempo del bucle es fijo, a 50 Hz, y no se puede cambiar. Lo que Ts afecta es la frecuencia
 *  de publicación del tópico que publica esos datos
 *
 * Me, un booleano que especifica el "modo de lectura". Si True, los valores de "ws" que se publican
 * 	se corresponden con las lecturas analógicas de los encoders y sus timeStamps en MICROSEGUNDOS.
 * 	Si False, se entiende que se desean los valores en velocidad angular de radianes/s y filtrados
 * 	ligeramente para reducir el ruido.
 *
 * par_rd, un flotante que permite actualizar el parámetro de radio de las ruedas
 *
 * par_wd es un flotante que permite actualizar el parámetro de longitud entre ruedas
 *
 *
 * Si en algún momento se cortase la transmisión de datos por un tiempo superior al especificado
 * por el timeOut, por default 2000 ms, los motores automáticamente se paran (sin frenarse, solo
 * se especifica un PWM igual al valor cero, por default 127.5)
 *
 */
#include <Arduino.h>

#include <ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

/*
 * Selección de los modos de operación, __ENCODER_ANALOG__ y __ENCODER_DIGITAL__, permiten
 * seleccionar entre los distintos tipos de toma de datos, __OUTPUT_SERIAL__ solo compila
 * con el arduino leonardo pero permite usar el Serial1 para comunicarse con los drivers,
 * de otra forma __OUTPUT_ANALOG__ deberá usarse
 */
#define __ENCODER_DIGITAL__
#define __OUTPUT_ANALOG__

#include "Motor/Motor_w_PID.cpp"
#include "HLayer/HL_Time.h"
#include "HLayer/structs.h"
#include "structs.h"

#include <Robot.h>

#ifdef __ENCODER_DIGITAL__
#include <AS5043.h>
AS5043Class encoders_spi;
#endif

#ifdef __OUTPUT_SERIAL__
#include <SaberSerial.cpp>
SaberSerialClass motors_serial;
#endif

/*
 * Aunque la clase Robot tiene asignado el nombre "robot", ciertas limitaciones en el uso de
 * punteros a funciones cuando se usan templates y la necesidad de asegurar un uso constante
 * de la memoria hacen difícil el hacer que la clase misma implemente TODOS los elementos
 * que le permitan funcionar de forma autocontenida sin romper el programa. Por ello, y sin
 * pensar en otras soluciones, se opta por hacer un archivo wrapper de métodos wrapper de lo que
 * necesita rosserial arduino para funcionar, y poner ese código fuera de éste archivo, por
 * uniformidad, consistencia y mantenimiento.
 *
 * Esto hace necesario el usar dos funciones, robotInit y robotPublish, que NO pertenecen a la
 * clase Robot, pero son las ÚNICAS que deben usarse para "interactuar" con las funciones, no
 * las propiedades (que modificamos manualmente en el setup) de la clase Robot
 *
 * Esto se puede resolver, por supuesto, pero probablemente requiera quitar el uso de templates
 * en los subscriptores, alterando indirectamente el código de rosserial, y no parece que hacer
 * eso sea una solución razonable. Una construcción cuidadosa de includes y cabeceras podría
 * también ayudar, pero no veo como eso, sin modificar rosserial, permita crear clases que pueda
 * publicay y subscribir de forma autocontenida
 */
RobotSpace::Robot robot;
ros::NodeHandle nh;

void setup()
{
	nh.initNode();
	robotInit();

#ifdef __ENCODER_DIGITAL__
	encoders_spi.begin();
#endif
#ifdef __OUTPUT_SERIAL__
	motors_serial.begin();
#endif

	// Especificación de datos de la clase RObot
	robot.timeOut = 2000;
	robot.modePassControl = true;
	robot.modePassEncoder = false;

	/**
	 * Los parámetros de cada uno de los motores se establecen por lista, por ahora
	 * la API no es enteramente consistente, daré la descripción línea por línea, pero
	 * está sujeta a cambios drásticos en el futuro
	 */

	// Constantes de control PID default, km es la constante del motor
	robot.motorR.pid.data_init.kp = 0.2;
	robot.motorR.pid.data_init.ki = 0.05;
	robot.motorR.pid.data_init.kd = 0.0;
	robot.motorR.pid.data_init.km = 0.1;

	robot.motorL.pid.data_init.kp = 0.2;
	robot.motorL.pid.data_init.ki = 0.05;
	robot.motorL.pid.data_init.kd = 0.0;
	robot.motorL.pid.data_init.km = 0.1;

	// Valores de constantes de filtrado kalman en una variable "degenerado"
	robot.motorR.encoder.kalman.data_kalman.A = 1.;
	robot.motorR.encoder.kalman.data_kalman.B = 1.;
	robot.motorR.encoder.kalman.data_kalman.C = 1.;
	robot.motorR.encoder.kalman.data_kalman.P_0 = 10.;
	robot.motorR.encoder.kalman.data_kalman.Q = 10.;
	robot.motorR.encoder.kalman.data_kalman.R = 10.;

	robot.motorL.encoder.kalman.data_kalman.A = 1.;
	robot.motorL.encoder.kalman.data_kalman.B = 1.;
	robot.motorL.encoder.kalman.data_kalman.C = 1.;
	robot.motorL.encoder.kalman.data_kalman.P_0 = 10.;
	robot.motorL.encoder.kalman.data_kalman.Q = 10.;
	robot.motorL.encoder.kalman.data_kalman.R = 10.;

	// Si false, el valor de lectura del encoder cambia de signo
	robot.motorR.encoder.dataIn.mode = true;
	robot.motorL.encoder.dataIn.mode = true;

	// Si false, el "sentido" del PWM alrededor del "cero" se invierte
	robot.motorR.pid.data_init.mode = true;
	robot.motorL.pid.data_init.mode = true;

	// El valor cero de salida PWM
	robot.motorR.pid.data_init.zeroRef = 127.5;
	robot.motorL.pid.data_init.zeroRef = 127.5;
	// Màximo término integral, unidades en PWM
	robot.motorR.pid.data_init.maxITerm = 15;
	robot.motorL.pid.data_init.maxITerm = 15;
	// Mínimo error para que actúe el término integral, en rad/s
	robot.motorR.pid.data_init.errWindup = 2;
	robot.motorL.pid.data_init.errWindup = 2;
	// Máxima salida del PWM alrededor del cero, es IGUAL al rango cero
	robot.motorR.pid.data_init.maxOut = 100;
	robot.motorL.pid.data_init.maxOut = 100;

#ifdef __ENCODER_DIGITAL__
	// El valor máximo de lectura del encoder, cambio máximo válido y pin de lectura
	// si es analógico, CSn si es digital
	robot.motorR.encoder.dataIn.range_enc = 1023;
	robot.motorR.encoder.dataIn.max_steep = 1023 / 4;
	robot.motorR.encoder.dataIn.pinHardware = 4;
	robot.motorL.encoder.dataIn.range_enc = 1023;
	robot.motorL.encoder.dataIn.max_steep = 1023 / 48;
	robot.motorL.encoder.dataIn.pinHardware = 5;
#else
	robot.motorR.encoder.dataIn.range_enc = 825;
	robot.motorR.encoder.dataIn.max_steep = 825 / 8;
	robot.motorR.encoder.dataIn.pinHardware = A0;
	robot.motorL.encoder.dataIn.range_enc = 1008;
	robot.motorL.encoder.dataIn.max_steep = 1008 / 8;
	robot.motorL.encoder.dataIn.pinHardware = A1;
#endif

#ifdef __OUTPUT_SERIAL__
	// Pin de salida del PWM si analógico, el número del motor (1 o 2)
	// si la interfaz es por Serial
	robot.motorR.pid.data_init.pinHardware = 1;
	robot.motorR.pid.data_init.pinHardware = 2;
#else
	robot.motorR.pid.data_init.pinHardware = 10;
	robot.motorR.pid.data_init.pinHardware = 11;
#endif

}

void loop()	{
	nh.spinOnce();
	robotPublish();
}
