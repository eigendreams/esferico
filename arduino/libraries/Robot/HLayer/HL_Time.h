// La separación en archivo de cabecera y el resto del código es necesarioa cuando
// se manejan el tiempo en Arduino con un wrapper

#ifndef HLS_TIME
#define HLS_TIME

namespace HLayerSpace
{
	unsigned long Time_milis_HL();
	unsigned long Time_micros_HL();
}

#endif
