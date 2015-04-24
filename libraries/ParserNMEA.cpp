#include "ParserNMEA.h"
/*
  Define os tipos de sentencas NMEA 0183 validas neste sistema. No caso
  deste sistemas, serao de interesse apenas essas quatro.
  
  GPRMC - Possui as informacoes mais basicas, necessarias para um GPS.
  GPZDA - Possui informacoes detalhadas sobre o tempo.
  GPGGA - Possui informacoes detalhadas sobre posicao.
  GPVTC - Possui informacoes detalhadas sobre velocidade.
*/
#define _GPRMC_TERM   "GPRMC"
#define _GPZDA_TERM   "GPZDA"
#define _GPVTG_TERM   "GPVTG"
#define _GPGGA_TERM   "GPGGA"


ParserNMEA::ParserNMEA()
:  _time(GPS_INVALID_TIME)
,  _day(GPS_INVALID_DATE)
,  _month(GPS_INVALID_DATE)
,  _year(GPS_INVALID_DATE)
,  _latitude(GPS_INVALID_ANGLE)
,  _longitude(GPS_INVALID_ANGLE)
,  _speed(GPS_INVALID_SPEED)
,  _course(GPS_INVALID_ANGLE)
,  _hdop(GPS_INVALID_HDOP)
,  _last_time_fix(GPS_INVALID_FIX_TIME)
,  _last_position_fix(GPS_INVALID_FIX_TIME)
,  _parity(0)
,  _is_checksum_term(false)
,  _sentence_type(_GPS_SENTENCE_OTHER)
,  _term_number(0)
,  _term_offset(0)
,  _gps_data_good(false)
#ifndef _GPS_NO_STATS
,  _encoded_characters(0)
,  _good_sentences(0)
,  _failed_checksum(0)
#endif
{
	_term[0] = '\0';
}

/*
   METODOS PUBLICOS
*/

/*
  Recebe caractere por caractere da sentenca e monta a variavel _term, 
  seguindo o protocolo NMEA0183. Retorna a informacao de validade da
  sentenca a cada caractere lido.
*/
bool ParserNMEA::encode(char c)
{
	bool valid_sentence = false;

	#ifndef _GPS_NO_STATS
	++_encoded_characters;
	#endif
	switch(c)
	{
		case ',': // Caractere que delimita os termos
		_parity ^= c;
		case '\r':
		case '\n':
		case '*':
		if (_term_offset < sizeof(_term))
		{
			_term[_term_offset] = 0;
			valid_sentence = term_complete();
		}
		++_term_number;
		_term_offset = 0;
		_is_checksum_term = c == '*';
		return valid_sentence;

		case '$': // Caractere que identifica o inicio de uma sentenca
		_term_number = _term_offset = 0;
		_parity = 0;
		_sentence_type = _GPS_SENTENCE_OTHER;
		_is_checksum_term = false;
		_gps_data_good = false;
		return valid_sentence;
	}

	// Caracteres ordinarios
	if (_term_offset < sizeof(_term) - 1)
	_term[_term_offset++] = c;
	if (!_is_checksum_term)
	_parity ^= c;

	return valid_sentence;
}

#ifndef _GPS_NO_STATS
void ParserNMEA::stats(unsigned long *chars, unsigned short *sentences, unsigned short *failed_cs)
{
	if (chars) *chars = _encoded_characters;
	if (sentences) *sentences = _good_sentences;
	if (failed_cs) *failed_cs = _failed_checksum;
}
#endif

/*
  Utilidades internas
*/

/*
  Converte um caractere hexadecimal para decimal.
*/
int ParserNMEA::from_hex(char a)
{
	if (a >= 'A' && a <= 'F')
	return a - 'A' + 10;
	else if (a >= 'a' && a <= 'f')
	return a - 'a' + 10;
	else
	return a - '0';
}

/*
  
*/
unsigned long ParserNMEA::parse_decimal()
{
	char *p = _term;
	bool isneg = *p == '-';
	if (isneg) ++p;
	unsigned long ret = 100UL * gpsatol(p);
	while (gpsisdigit(*p)) ++p;
	if (*p == '.')
	{
		if (gpsisdigit(p[1]))
		{
			ret += 10 * (p[1] - '0');
			if (gpsisdigit(p[2]))
			ret += p[2] - '0';
		}
	}
	return isneg ? -ret : ret;
}

// Parse a string in the form ddmm.mmmmmmm...
unsigned long ParserNMEA::parse_degrees()
{
	char *p;
	unsigned long left_of_decimal = gpsatol(_term);
	unsigned long hundred1000ths_of_minute = (left_of_decimal % 100UL) * 100000UL;
	for (p=_term; gpsisdigit(*p); ++p);
	if (*p == '.')
	{
		unsigned long mult = 10000;
		while (gpsisdigit(*++p))
		{
			hundred1000ths_of_minute += mult * (*p - '0');
			mult /= 10;
		}
	}
	return (left_of_decimal / 100) * 1000000 + (hundred1000ths_of_minute + 3) / 6;
}

#define COMBINE(sentence_type, term_number) (((unsigned)(sentence_type) << 5) | term_number)

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
bool ParserNMEA::term_complete()
{
	if (_is_checksum_term)
	{
		byte checksum = 16 * from_hex(_term[0]) + from_hex(_term[1]);
		if (checksum == _parity)
		{
			if (_gps_data_good)
			{
				#ifndef _GPS_NO_STATS
				++_good_sentences;
				#endif
				_last_time_fix = _new_time_fix;
				_last_position_fix = _new_position_fix;

				switch(_sentence_type)
				{
					case _GPS_SENTENCE_GPRMC:
					_time      = _new_time;
					_latitude  = _new_latitude;
					_longitude = _new_longitude;
					_speed     = _new_speed;
					break;
					case _GPS_SENTENCE_GPVTG:
					_speed     = _new_speed;
					break;
					case _GPS_SENTENCE_GPZDA:
				    _time      = _new_time;
					_day      = _new_day;
					_month    = _new_month;
					_year     = _new_year;
					break;
					case _GPS_SENTENCE_GPGGA:
					_time      = _new_time;
					_latitude  = _new_latitude;
					_longitude = _new_longitude;
					_hdop      = _new_hdop;
					break;
				}

				return true;
			}
		}

		#ifndef _GPS_NO_STATS
		else
		++_failed_checksum;
		#endif
		return false;
	}

	// the first term determines the sentence type
	if (_term_number == 0)
	{
		if (!gpsstrcmp(_term, _GPZDA_TERM))
		_sentence_type = _GPS_SENTENCE_GPZDA;
		else if (!gpsstrcmp(_term, _GPGGA_TERM))
		_sentence_type = _GPS_SENTENCE_GPGGA;
		else if (!gpsstrcmp(_term, _GPVTG_TERM))
		_sentence_type = _GPS_SENTENCE_GPVTG;
		else
		_sentence_type = _GPS_SENTENCE_OTHER;
		return false;
	}

	if (_sentence_type != _GPS_SENTENCE_OTHER && _term[0])
	switch(COMBINE(_sentence_type, _term_number))
	{
		case COMBINE(_GPS_SENTENCE_GPZDA, 1): // Time in ZDA
		_new_time = parse_decimal();
		_new_time_fix = millis();
		break;
		case COMBINE(_GPS_SENTENCE_GPRMC, 2): // GPRMC validity
		_gps_data_good = _term[0] == 'A';
		break;
		case COMBINE(_GPS_SENTENCE_GPRMC, 3): // Latitude
	    _new_latitude = parse_degrees();
		_new_position_fix = millis();
		break;
		case COMBINE(_GPS_SENTENCE_GPGGA, 2):
		_new_latitude = parse_degrees();
		_new_position_fix = millis();
		break;
		case COMBINE(_GPS_SENTENCE_GPRMC, 4): // N/S
		if (_term[0] == 'S')
		_new_latitude = -_new_latitude;
		break;
		case COMBINE(_GPS_SENTENCE_GPGGA, 3):
		if (_term[0] == 'S')
		_new_latitude = -_new_latitude;
		break;
		case COMBINE(_GPS_SENTENCE_GPGGA, 4):
		_new_longitude = parse_degrees();
		break;
		case COMBINE(_GPS_SENTENCE_GPGGA, 5):
		if (_term[0] == 'W')
		_new_longitude = -_new_longitude;
		break;
		case COMBINE(_GPS_SENTENCE_GPVTG, 5): // Speed (GPVTG)
		_new_speed = parse_decimal();
		break;
		case COMBINE(_GPS_SENTENCE_GPRMC, 8): // Course (GPRMC)
		_new_course = parse_decimal();
		break;
		case COMBINE(_GPS_SENTENCE_GPZDA, 2): // Date/day (GPZDA)
		_new_day = (unsigned char)atoi(_term);
		break;
		case COMBINE(_GPS_SENTENCE_GPZDA, 3): // Date/month (GPZDA)
		_new_month = (unsigned char)atoi(_term);
		break;
		case COMBINE(_GPS_SENTENCE_GPZDA, 4): // Date/year (GPZDA)
		 _new_year = parse_decimal() / 100;
		break;
		case COMBINE(_GPS_SENTENCE_GPGGA, 6): // Fix data (GPGGA)
		_gps_data_good = _term[0] > '0';
		break;
		case COMBINE(_GPS_SENTENCE_GPGGA, 8): // HDOP
		_new_hdop = parse_decimal();
		break;
	}

	return false;
}

/*
  
*/
long ParserNMEA::gpsatol(const char *str)
{
	long ret = 0;
	while (gpsisdigit(*str))
	ret = 10 * ret + *str++ - '0';
	return ret;
}

int ParserNMEA::gpsstrcmp(const char *str1, const char *str2)
{
	while (*str1 && *str1 == *str2)
	++str1, ++str2;
	return *str1;
}

/*
	Retorna a distancia entre duas coordenadas
	
	As infomacoes referentes ao calculo realizado neste metodo foram extraidas da 
	seguinte pagina,

	http://www.movable-type.co.uk/scripts/latlong.html
	
*/
float ParserNMEA::distance_between (float lat1, float long1, float lat2, float long2)
{

	float delta = radians(long1-long2);
	float sdlong = sin(delta);
	float cdlong = cos(delta);
	lat1 = radians(lat1);
	lat2 = radians(lat2);
	float slat1 = sin(lat1);
	float clat1 = cos(lat1);
	float slat2 = sin(lat2);
	float clat2 = cos(lat2);
	delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
	delta = sq(delta);
	delta += sq(clat2 * sdlong);
	delta = sqrt(delta);
	float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
	delta = atan2(delta, denom);
	return delta * 6372795;
}

/*
	A partir das coordenadas, passada e presente, espaçadas no tempo pelo periodo de
    atualizacao das informacoes de posicao do receptor, este metodo calcula um angulo
	que carrega informacao relacionada ao curso do sistema de navegacao.	
*/

float ParserNMEA::course_to (float lat1, float long1, float lat2, float long2)
{

	float dlon = radians(long2-long1);
	lat1 = radians(lat1);
	lat2 = radians(lat2);
	float a1 = sin(dlon) * cos(lat2);
	float a2 = sin(lat1) * cos(lat2) * cos(dlon);
	a2 = cos(lat1) * sin(lat2) - a2;
	a2 = atan2(a1, a2);
	if (a2 < 0.0)
	{
		a2 += TWO_PI;
	}
	return degrees(a2);
}

/*
	Baseado em uma informacao de curso, previamente calculada, este metodo tem a funcao
	de determinar a direcao cardeal do sistema de navegacao.
*/
const char *ParserNMEA::cardinal (float course)
{
	static const char* directions[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};

	int direction = (int)((course + 11.25f) / 22.5f);
	return directions[direction % 16];
}

// Retorna o curso como float.
float ParserNMEA::f_course()
{
	return _course == GPS_INVALID_ANGLE ? GPS_INVALID_F_ANGLE : _course / 100.0;
}

// Retorna a velocidade, em nos, como float.
float ParserNMEA::f_vtg_speed_knots()
{
	return _speed == GPS_INVALID_SPEED ? GPS_INVALID_F_SPEED : _speed / 100.0;
}

// Retorna a velocidade, em m/h, como float.
float ParserNMEA::f_vtg_speed_mph()
{
	float sk = f_vtg_speed_knots();
	return sk == GPS_INVALID_F_SPEED ? GPS_INVALID_F_SPEED : _GPS_MPH_PER_KNOT * sk;
}

// Retorna a velocidade, em m/s, como float.
float ParserNMEA::f_vtg_speed_mps()
{
	float sk = f_vtg_speed_knots();
	return sk == GPS_INVALID_F_SPEED ? GPS_INVALID_F_SPEED : _GPS_MPS_PER_KNOT * sk;
}

// Retorna a velocidade, em km/h, como float.
float ParserNMEA::f_vtg_speed_kmph()
{
	float sk = f_vtg_speed_knots();
	return sk == GPS_INVALID_F_SPEED ? GPS_INVALID_F_SPEED : _GPS_KMPH_PER_KNOT * sk;
}

// Retorna a latitude, poveniente da sentenca do tipo GGA, como float.
float ParserNMEA::f_gga_lat()
{
	return (_latitude/1000000.0);

}

// Retorna a longitude, proveniente da sentenca GGA, como float.
float ParserNMEA::f_gga_lon()
{
	return (_longitude/1000000.0);
}

// Retorna a latitude, proveniente da sentenca do tipo GGA, como long.
long ParserNMEA::gga_lat()
{
	return _latitude;
}

// Retorna a longitude, proveniente da sentenca do tipo GGA, como long.
long ParserNMEA::gga_lon()
{
	long tmp = floor(_longitude / 100);
	long ret = tmp + (_longitude - tmp * 100) / 60;
	return ret;
}

// Retorna o ano, provenitente da sentenca do tipo ZDA, como int.
int ParserNMEA::zda_year()
{
	return _new_year;
}

// Retorna o mes, proveniente da sentenca do tipo ZDA, como int.
int ParserNMEA::zda_month()
{
	return _new_month;
}

// Retorna o dia, proveniente da sentenca do tipo ZDA, como int.
int ParserNMEA::zda_day()
{
	return _new_day;
}

// retorna o tempo, proveniente da sentenca do tipo ZDA, como long sem sinal.

unsigned long ParserNMEA::zda_time()
{
	return _time;
}
const float ParserNMEA::GPS_INVALID_F_ANGLE = 1000.0;
const float ParserNMEA::GPS_INVALID_F_SPEED = -1.0;
