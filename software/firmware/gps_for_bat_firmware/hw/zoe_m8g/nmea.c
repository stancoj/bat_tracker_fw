/*
 * nmea.c
 *
 *  Created on: 29. 10. 2018
 *      Author: David Rau
 */

#ifndef NMEA_C_
#define NMEA_C_

#include "nmea.h"

ZDA_Data_s proceedZDAparsing(uint8_t *buffer)
{
	uint8_t index; int8_t dummy;
	ZDA_Data_s data = ZDA_DATA_INIT;
	index = parseHeader(buffer, 0);
	index = parseTime(buffer, &data.time, &dummy,  index);
	index = parseDay(buffer, &data.date, index);
	index = parseMonth(buffer, &data.date, index);
	index = parseYear(buffer, &data.date, index);
	return data;
}

RMC_Data_s proceedRMCparsing(uint8_t *buffer)
{
    uint8_t index;
    RMC_Data_s data = RMC_DATA_INIT;

    index = parseHeader(buffer, 0);
    index = parseTime(buffer, &data.time, &data.timeValid,  index);
    index = parseStatus(buffer, &data.status, &data.gpsValid, index);
    index = parseLatitude(buffer, &data.latitude, index);
	index = parseLongitude(buffer, &data.longitude, index);
	index = parseGroundSpeed(buffer, &data.speed, index);
	index = parseTrackAngle(buffer, &data.trackAngle, index);
	index = parseDate(buffer, &data.date, index);
	index = parseMagneticVariation(buffer, &data.magneticVariation, index);
	index = parseEmpty(buffer, index);
	index = parsePosMode(buffer, &data.posMode, index);
    return data;
}

uint8_t parseTrackAngle(uint8_t * buffer, float *data, uint8_t iComma)
{
    uint16_t angleFraction = 0;
    uint16_t angleFull = 0, denumerator = 1;
    int8_t signum = 1;

    if(buffer[iComma+1] == ',')
	{
		return iComma+1;
	}

    if(buffer[iComma+1]=='-')
    {
        signum = -1;
        iComma++;
    }
    for(iComma++; buffer[iComma] != '.'; iComma++)
    {
        angleFull = angleFull * 10 + ((buffer[iComma])-'0');
    }
    for(iComma++; buffer[iComma] != ','; iComma++)
    {
        angleFraction = angleFraction * 10 + ((buffer[iComma])-'0');
        denumerator *= 10;
    }
    *data = signum * ((float)angleFraction/denumerator + angleFull);
    return iComma;
}

uint8_t parseDate(uint8_t * buffer, NMEA_Date_s *data, uint8_t iComma)
{
	if(buffer[iComma+1] == ',')
	{
		return iComma + 1;
	}

	data->year = (buffer[iComma + 5] - '0') * (short)10 + buffer[iComma + 6] - '0' + 2000;
	data->month = (buffer[iComma + 3] - '0') * 10 + buffer[iComma + 4] - '0';
	data->day = (buffer[iComma + 1] - '0') * 10 + buffer[iComma + 2] - '0';
    return iComma + 7;
}

uint8_t parseDay(uint8_t * buffer, NMEA_Date_s *data, uint8_t iComma)
{
	if(buffer[iComma+1] == ',')
	{
		return iComma + 1;
	}

	data->day = (buffer[iComma + 1] - '0') * 10 + buffer[iComma + 2] - '0';
	return iComma + 3;
}

uint8_t parseMonth(uint8_t * buffer, NMEA_Date_s *data, uint8_t iComma)
{
	if(buffer[iComma+1] == ',')
	{
		return iComma + 1;
	}

	data->month = (buffer[iComma + 1] - '0') * 10 + buffer[iComma + 2] - '0';
	return iComma + 3;
}

uint8_t parseYear(uint8_t * buffer, NMEA_Date_s *data, uint8_t iComma)
{
	if(buffer[iComma+1] == ',')
	{
		return iComma + 1;
	}

	data->year = (buffer[iComma + 1] - '0') * 1000 + (buffer[iComma + 2] - '0') * 100 + (buffer[iComma + 3] - '0') * 10 + buffer[iComma + 4] - '0';
	return iComma + 5;
}

uint8_t parseMagneticVariation(uint8_t * buffer, float *data, uint8_t iComma)
{
    uint8_t angleFraction = 0;
    uint16_t angleFull = 0;
    uint16_t denumerator = 1;
    int8_t signum = 1;


    iComma++;
    if(buffer[iComma] == ',')
	{
		return iComma;
	}

    for(; buffer[iComma] != '.'; iComma++)
    {
        angleFull = angleFull * 10 + ((buffer[iComma])-'0');
    }
    iComma++;
    for(; buffer[iComma] != ','; iComma++)
    {
        angleFraction = angleFraction * 10 + ((buffer[iComma])-'0');
        denumerator *= 10;
    }
    if(buffer[iComma + 2] == 'W')
    {
        signum = -1;
    }
    *data = signum * ((float)angleFraction/denumerator + angleFull);
    iComma++;

    return iComma;
}

//***************** Header **********************
uint8_t parseHeader(uint8_t *buffer, uint8_t iComma)
{
    uint8_t i = iComma;

    for(i = iComma; 1; i++)
    {
        if (buffer[i] == ',')
        {
            iComma=i;
            break;
        }
    }

    return iComma;
}

uint8_t parseOpMode(uint8_t *buffer, Eauto_manual_2D_3D_Fix * data, uint8_t iComma)
{
	if(buffer[iComma+1] == ',')
	{
		return iComma+1;
	}

	*data = (Eauto_manual_2D_3D_Fix)buffer[iComma + 1];

	return iComma + 2;
}

uint8_t parseNavMode(uint8_t *buffer, E2D_3D_Fix * data, uint8_t iComma)
{
	if(buffer[iComma+1] == ',')
	{
		return iComma+1;
	}

	*data = (E2D_3D_Fix)buffer[iComma + 1];

	return iComma + 2;
}

//***************** Status **********************
uint8_t parseStatus(uint8_t *buffer, uint8_t *data, int8_t *valid, uint8_t iComma)
{
	if(buffer[iComma+1] == ',')
	{
		return iComma+1;
	}

	*data = buffer[iComma + 1];

    if(*data == 'A')
    {
    	*valid = 1;
    }

    return iComma + 2;
}

uint8_t parsePosMode(uint8_t *buffer, uint8_t *data, uint8_t iComma)
{
	if(buffer[iComma+1] == ',')
	{
		return iComma+1;
	}

	*data = buffer[iComma + 1];

    return iComma + 2;
}

//***************** Time **********************
uint8_t parseTime(uint8_t *buffer, NMEA_Time_s *data, int8_t* timeValid,  uint8_t iComma)
{
    uint8_t i = iComma;

    if(buffer[i+1] == ',')
    {
    	*timeValid = 0;
    	return iComma+1;
    }

    data->sec=(buffer[i+5]-'0')*10 + buffer[i+6]-'0';
    data->min=(buffer[i+3]-'0')*10 + buffer[i+4]-'0';
    data->hour= (buffer[i+1]  -'0')*10 + buffer[i+2]-'0';

    *timeValid = 1;
    for(i = iComma + 1; 1; i++)
    {
        if (buffer[i] == ',')
        {
            iComma=i;
            break;
        }
    }
    data->utcDaySeconds = (long)data->hour*60*60 + (long)data->min*60 + data->sec;
    return iComma;
}

uint8_t parseLatitude(uint8_t *buffer, NMEA_Latitude_s *data, uint8_t iComma)
{
    uint8_t degrees, minutes;
    unsigned long minuteFraction = 0;
    unsigned long minuteFractionDenominator = 1;
    float minuteFractionF = 0;

    int32_t microdegrees = 0;

    if(buffer[iComma+1] == ',')
	{
		return iComma+1;
	}

    degrees=(buffer[iComma+1]-'0')*10+buffer[iComma+2]-'0';
    minutes=(buffer[iComma+3]-'0')*10+buffer[iComma+4]-'0';

    //////////////////////////nove////////////////////////////////////////////////
    microdegrees = degrees*MICRODEGREES_MULTIPLY;

    microdegrees += (minutes*MICRODEGREES_MULTIPLY)/60;
    //////////////////////////////////////////////////////////////////////////////


    for( iComma += 6; buffer[iComma]!= ','; iComma++)
    {
        minuteFraction = (minuteFraction * 10 + buffer[iComma] - '0');
        minuteFractionDenominator = minuteFractionDenominator * 10;
    }
    minuteFractionF = (float)minuteFraction / minuteFractionDenominator;
    data->latitude = degrees + (minuteFractionF + minutes) / 60;

    data->latitudeDegMin = degrees * 100 + minutes;
    data->latitudeDegMinFraction = (uint32_t) minuteFraction * 10000/minuteFractionDenominator;

    //////////////////////////nove////////////////////////////////////////////////
//    minuteFraction /= 60;
//
//    if (minuteFractionDenominator >= MICRODEGREES_MULTIPLY)
//    {
//    	minuteFractionDenominator /= MICRODEGREES_MULTIPLY;
//    	microdegrees += minuteFraction / minuteFractionDenominator;
//    }
//    else
//    {
//    	minuteFractionDenominator = MICRODEGREES_MULTIPLY/minuteFractionDenominator;
//    	microdegrees += minuteFraction * minuteFractionDenominator;
//    }

    int64_t microDegreesMinuteFraction = (int64_t)minuteFraction * MICRODEGREES_MULTIPLY/(minuteFractionDenominator * 60);
    microdegrees += microDegreesMinuteFraction;
    //////////////////////////////////////////////////////////////////////////////


    if(buffer[iComma+1] =='S')
    {
    	data->latitude = -data->latitude;
        microdegrees *= -1;
    }

    data->latitudeMicroDegree = microdegrees;

    return iComma+2;
}

//***************** Longitude **********************
uint8_t parseLongitude(uint8_t *buffer, NMEA_Longitude_s *data, uint8_t iComma)
{
    uint8_t degrees, minutes;
    unsigned long minuteFraction = 0;
    unsigned long minuteFractionDenominator = 1;
    float minuteFractionF = 0;

    int32_t microdegrees = 0;

    if(buffer[iComma+1] == ',')
	{
		return iComma+1;
	}

    degrees=(buffer[iComma+1]-'0')*100 + ( buffer[iComma+2] - '0') * 10 + buffer[iComma+3] -'0';
    minutes=(buffer[iComma+4]-'0')*10+buffer[iComma+5] - '0';

    //////////////////////////nove////////////////////////////////////////////////
    microdegrees = ((buffer[iComma+1]-'0')*100 + ( buffer[iComma+2] - '0') * 10
    		+ buffer[iComma+3] -'0')*MICRODEGREES_MULTIPLY;

    microdegrees += (minutes*MICRODEGREES_MULTIPLY)/60;
    //////////////////////////////////////////////////////////////////////////////


    for( iComma += 7; buffer[iComma]!= ','; iComma++)
    {
        minuteFraction = (minuteFraction * 10 + buffer[iComma] - '0');
        minuteFractionDenominator = minuteFractionDenominator * 10;
    }


    minuteFractionF = (float)minuteFraction / minuteFractionDenominator;
    data->longitude = degrees + (minuteFractionF + minutes) / 60;


    //////////////////////////nove////////////////////////////////////////////////
//    minuteFraction /= 60;
//
//    if (minuteFractionDenominator >= MICRODEGREES_MULTIPLY)
//    {
//    	minuteFractionDenominator /= MICRODEGREES_MULTIPLY;
//    	microdegrees += minuteFraction / minuteFractionDenominator;
//    }
//    else
//    {
//    	minuteFractionDenominator = MICRODEGREES_MULTIPLY/minuteFractionDenominator;
//    	microdegrees += minuteFraction * minuteFractionDenominator;
//    }

    int64_t microDegreesMinuteFraction = (int64_t)minuteFraction * MICRODEGREES_MULTIPLY/(minuteFractionDenominator * 60);
    microdegrees += microDegreesMinuteFraction;

    //////////////////////////////////////////////////////////////////////////////

    data->longitudeDegMin = degrees * 100 + minutes;
    data->longitudeDegMinFraction = (uint32_t) minuteFraction * 10000/minuteFractionDenominator;

    if(buffer[iComma+1] =='W')
    {
        microdegrees = (360*MICRODEGREES_MULTIPLY) - microdegrees;

        data->longitude *= -1;
    }

    data->longitudeMicroDegree = microdegrees;

    return iComma+2;
}

uint8_t parseGroundSpeed(uint8_t * buffer, NMEA_Speed_s *data, uint8_t iComma)
{
    uint8_t speedFraction = 0;
    uint16_t speedFull = 0, denumerator = 1;
    int8_t signum = 1;

    float speedMPS = 0;

    if(buffer[iComma+1] == ',')
	{
		return iComma+1;
	}

    if(buffer[iComma+1]=='-')
    {
        signum = -1;
        iComma++;
    }
    for(iComma++; buffer[iComma] != '.'; iComma++)
    {
        speedFull = speedFull * 10 + ((buffer[iComma])-'0');
    }
    for(iComma++; buffer[iComma] != ','; iComma++)
    {
        speedFraction = speedFraction * 10 + ((buffer[iComma])-'0');
        denumerator *= 10;
    }

    data->speedKmPH = 1.852 * signum * ((float)speedFraction/denumerator + speedFull);

    speedMPS = data->speedKmPH / 3.6;

    data->speedMPS10 = (uint32_t)(speedMPS * 10);

    return iComma;
}

GGA_Data_s proceedGGAparsing(uint8_t * buffer)
{
    uint8_t index;
    GGA_Data_s data = GGA_DATA_INIT;
    index = parseHeader(buffer, 0);
    index = parseTime(buffer, &data.time, &data.timeValid,  index);
    index = parseLatitude(buffer, &data.latitude, index);
    index = parseLongitude(buffer, &data.longitude, index);
    index = parseQuality(buffer, &data.quality, &data.gpsValid, index);
    index = parseSatelitesTracked(buffer, &data.satTracked, index);
    index = parseDilutionOfPrecision(buffer, &data.horizontalDilutionOfPrecision, index);
    index = parseAltitudeMeters(buffer, &data.altitude, index);
    index = parseEmpty(buffer, index);
    index = parseHeightOfGeoid(buffer, &data.altitude, index);
    return data;
}

GSA_Data_s proceedGSAparsing(uint8_t * buffer)
{
    uint8_t index;

	GSA_Data_s data = GSA_DATA_INIT;

	index = parseHeader(buffer, 0);
	index = parseOpMode(buffer, &data.eAutoManualFix, index);
	index = parseNavMode(buffer, &data.e2D3Dfix, index);

    if(data.e2D3Dfix > eNofix)
    {
        index = parseActiveSatelliteList(buffer, &data.satellite, index);
        index = parseDilutionOfPrecision(buffer, &data.positionDilutionOfPrecision, index);
        index = parseDilutionOfPrecision(buffer, &data.horizontalDilutionOfPrecision, index);
        index = parseDilutionOfPrecision(buffer, &data.verticalDilutionOfPrecision, index);
    }

    return data;
}

uint8_t parseEmpty(uint8_t *buffer, uint8_t iComma)
{
	uint8_t i;

	for(i = iComma + 1; 1; i++)
	{
		if (buffer[i] == ',')
		{
			iComma=i;
			break;
		}
	}
	return iComma;
}

uint8_t parseActiveSatelliteList(uint8_t* buffer, NMEA_Satellite_s *data, uint8_t iComma)
{
    uint8_t commaCount;

    data->activeSatelliteCount = 0;
    for(commaCount = 0; commaCount < 12; commaCount++)
    {
        if(buffer[iComma++] == ',')
        {
            continue;
        }
        else
        {
        	data->activeSatelliteList[data->activeSatelliteCount++]
                 = (buffer[iComma-1] - '0') * 10 + buffer[iComma] - '0';
            iComma += 2;
        }
    }
    return iComma;
}

uint8_t parsePositionDilutionOfPrecision(uint8_t* buffer, float *data, uint32_t iComma)
{
    uint8_t dilutionFraction1;
    uint16_t dilutionFull = 0;
    int8_t signum = 1;

    if(buffer[iComma+1] == ',')
	{
		return iComma+1;
	}

    if(buffer[iComma]=='-')
    {
        signum = -1;
        iComma++;
    }
    for(; buffer[iComma] != '.'; iComma++)
    {
        dilutionFull = dilutionFull * 10 + ((buffer[iComma])-'0');
    }
    iComma++;
    dilutionFraction1 = buffer[iComma] - '0';
    iComma++;
    *data = signum * ((float)dilutionFraction1/10 + dilutionFull);

//    data->gpsDOP = (int8_t) (data->dilutionOfPrecision*10);

    return iComma+1;
}

uint8_t parseVerticalDilutionOfPrecision(uint8_t* buffer, float *data, uint32_t iComma)
{
    uint8_t dilutionFraction1;
    uint16_t dilutionFull = 0;
    int8_t signum = 1;

    if(buffer[iComma+1] == ',')
	{
		return iComma+1;
	}

    if(buffer[iComma]=='-')
    {
        signum = -1;
        iComma++;
    }
    for(; buffer[iComma] != '.'; iComma++)
    {
        dilutionFull = dilutionFull * 10 + ((buffer[iComma])-'0');
    }
    iComma++;
    dilutionFraction1 = buffer[iComma] - '0';
    iComma++;
    *data = signum * ((float)dilutionFraction1/10 + dilutionFull);
    return iComma;

}

uint8_t parseQuality(uint8_t *buffer, uint8_t *data, int8_t *valid, uint8_t iComma)
{
	*data = buffer[iComma + 1] - '0';

	if(*data >= 1 && *data <= 6)
	{
		*valid = 1;
	}
	else
	{
		*valid = 0;
	}
	return iComma + 2;
}

//************ Fix quality ********
//uint8_t validityFixFromGGA(uint8_t *buffer, GGA_Data_s *data)
//{
//    uint8_t i;
//    for (i=26;i < MAXIMUM_GPS_BUFFER_LENGTH;i++)
//    {
//        if ((buffer[i]== 'W')||(buffer[i]== 'E')) // find one field before the Fixquality
//        {
//            break;
//        }
//    }
//    data->qualityFixFromGGA = buffer[i+2]-'0';
//
//    if((data->qualityFixFromGGA >= 1)&&(data->qualityFixFromGGA <= 8))
//    {
//        return 1;//if Valid return 1
//    }
//    else
//    {
//         return 0;
//    }
//}

//***************** Time **********************
//uint8_t parseTimeFromGGA(uint8_t * buffer, GGA_Data_s *data, uint8_t iComma)
//{
//    uint8_t i = iComma;
//    data->sec=(buffer[i+4]-'0')*10 + buffer[i+5]-'0';
//    data->min=(buffer[i+2]-'0')*10 + buffer[i+3]-'0';
//    data->hour= (buffer[i]  -'0')*10 + buffer[i+1]-'0';
//    for(i=iComma;1;i++)
//    {
//        if (buffer[i] == ',')
//        {
//            iComma=i;
//            break;
//        }
//    }
//    data->utcDaySeconds = (long)data->hour*60*60 + (long)data->min*60 + data->sec;
//    return iComma;
//}

//***************** Latitude **********************
//uint8_t parseLatitudeFromGGA(uint8_t *buffer, GGA_Data_s *data, uint8_t iComma)
//{
//    uint8_t degrees, minutes;
//    unsigned long minuteFraction = 0;
//    unsigned long minuteFractionDenominator = 1;
//    float minuteFractionF = 0;
//
//    int32_t microdegrees = 0;
//
//    degrees=(buffer[iComma+1]-'0')*10+buffer[iComma+2]-'0';
//    minutes=(buffer[iComma+3]-'0')*10+buffer[iComma+4]-'0';
//
//    //////////////////////////nove////////////////////////////////////////////////
//    microdegrees = degrees*MICRODEGREES_MULTIPLY;
//
//    microdegrees += (minutes*MICRODEGREES_MULTIPLY)/60;
//    //////////////////////////////////////////////////////////////////////////////
//
//
//    for( iComma += 6; buffer[iComma]!= ','; iComma++)
//    {
//        minuteFraction = (minuteFraction * 10 + buffer[iComma] - '0');
//        minuteFractionDenominator = minuteFractionDenominator * 10;
//    }
//    minuteFractionF = (float)minuteFraction / minuteFractionDenominator;
//    data->latitude = degrees + (minuteFractionF + minutes) / 60;
//
//    data->latitudeDegMin = degrees * 100 + minutes;
//    data->latitudeDegMinFraction = (uint32_t) minuteFraction * 10000/minuteFractionDenominator;
//
//    //////////////////////////nove////////////////////////////////////////////////
////    minuteFraction /= 60;
////
////    if (minuteFractionDenominator >= MICRODEGREES_MULTIPLY)
////    {
////    	minuteFractionDenominator /= MICRODEGREES_MULTIPLY;
////    	microdegrees += minuteFraction / minuteFractionDenominator;
////    }
////    else
////    {
////    	minuteFractionDenominator = MICRODEGREES_MULTIPLY/minuteFractionDenominator;
////    	microdegrees += minuteFraction * minuteFractionDenominator;
////    }
//
//    int64_t microDegreesMinuteFraction = (int64_t)minuteFraction * MICRODEGREES_MULTIPLY/(minuteFractionDenominator * 60);
//    microdegrees += microDegreesMinuteFraction;
//    //////////////////////////////////////////////////////////////////////////////
//
//
//    if(buffer[iComma+1] =='S')
//    {
//    	data->latitude = -data->latitude;
//        microdegrees *= -1;
//    }
//
//    data->latitudeMicroDegree = microdegrees;
//
//    return iComma+2;
//}
//
////***************** Longitude **********************
//uint8_t parseLongitudeFromGGA(uint8_t * buffer, GGA_Data_s *data, uint8_t iComma)
//{
//    uint8_t degrees, minutes;
//    unsigned long minuteFraction = 0;
//    unsigned long minuteFractionDenominator = 1;
//    float minuteFractionF = 0;
//
//    int32_t microdegrees = 0;
//
//    degrees=(buffer[iComma+1]-'0')*100 + ( buffer[iComma+2] - '0') * 10 + buffer[iComma+3] -'0';
//    minutes=(buffer[iComma+4]-'0')*10+buffer[iComma+5] - '0';
//
//    //////////////////////////nove////////////////////////////////////////////////
//    microdegrees = ((buffer[iComma+1]-'0')*100 + ( buffer[iComma+2] - '0') * 10
//    		+ buffer[iComma+3] -'0')*MICRODEGREES_MULTIPLY;
//
//    microdegrees += (minutes*MICRODEGREES_MULTIPLY)/60;
//    //////////////////////////////////////////////////////////////////////////////
//
//
//    for( iComma += 7; buffer[iComma]!= ','; iComma++)
//    {
//        minuteFraction = (minuteFraction * 10 + buffer[iComma] - '0');
//        minuteFractionDenominator = minuteFractionDenominator * 10;
//    }
//
//
//    minuteFractionF = (float)minuteFraction / minuteFractionDenominator;
//    data->longitude = degrees + (minuteFractionF + minutes) / 60;
//
//
//    //////////////////////////nove////////////////////////////////////////////////
////    minuteFraction /= 60;
////
////    if (minuteFractionDenominator >= MICRODEGREES_MULTIPLY)
////    {
////    	minuteFractionDenominator /= MICRODEGREES_MULTIPLY;
////    	microdegrees += minuteFraction / minuteFractionDenominator;
////    }
////    else
////    {
////    	minuteFractionDenominator = MICRODEGREES_MULTIPLY/minuteFractionDenominator;
////    	microdegrees += minuteFraction * minuteFractionDenominator;
////    }
//
//    int64_t microDegreesMinuteFraction = (int64_t)minuteFraction * MICRODEGREES_MULTIPLY/(minuteFractionDenominator * 60);
//    microdegrees += microDegreesMinuteFraction;
//
//    //////////////////////////////////////////////////////////////////////////////
//
//    data->longitudeDegMin = degrees * 100 + minutes;
//    data->longitudeDegMinFraction = (uint32_t) minuteFraction * 10000/minuteFractionDenominator;
//
//    if(buffer[iComma+1] =='W')
//    {
//        microdegrees = (360*MICRODEGREES_MULTIPLY) - microdegrees;
//
//        data->longitude *= -1;
//    }
//
//    data->longitudeMicroDegree = microdegrees;
//
//    return iComma+2;
//}

//************Number of satellites being tracked***********
uint8_t parseSatelitesTracked(uint8_t * buffer, uint8_t *data, uint8_t iComma)
{

    if(buffer[iComma+2] == ',')
    {
    	*data = (buffer[iComma+1]-'0');
        iComma = iComma+2;
    }
    else
    {
    	*data = (buffer[iComma+1]-'0')*10+(buffer[iComma+2]-'0');
        iComma = iComma+3;
    }
    return iComma;
}


uint8_t parseDilutionOfPrecision(uint8_t * buffer, float *data, uint8_t iComma)
{
    uint8_t dilutionFraction = 0;
    uint16_t dilutionFull = 0;
    uint16_t denominator = 1;
    int8_t signum = 1;

    if(buffer[iComma+1] == ',')
	{
		return iComma+1;
	}

    if(buffer[iComma+1]=='-')
    {
        signum = -1;
        iComma++;
    }
    for(iComma++; buffer[iComma] != '.'; iComma++)
    {
        dilutionFull = dilutionFull * 10 + ((buffer[iComma])-'0');
    }
    for(iComma++; buffer[iComma] != ','; iComma++)
    {
        dilutionFraction = dilutionFraction * 10 + ((buffer[iComma])-'0');
        denominator *=10;
    }
    //iComma++;
    *data = signum * ((float)dilutionFraction/denominator + dilutionFull);
    return iComma;
}

//************Horizontal dilution of position***********
uint8_t parseAltitudeMeters(uint8_t * buffer, NMEA_Altitude_s *data, uint8_t iComma)
{
//$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\n
    uint8_t altitudeFraction1;
    uint16_t altitudeFull = 0;
    int8_t signum = 1;

    if(buffer[iComma+1]=='-')
    {
        signum = -1;
        iComma++;
    }

    for(iComma++; buffer[iComma] != '.'; iComma++)
    {
        altitudeFull = altitudeFull * 10 + (buffer[iComma] - '0');
    }

    data->altitudeUInt = signum * altitudeFull;

    iComma++;
    altitudeFraction1 = buffer[iComma] - '0';
    iComma++;
    data->altitude = signum * ((float)altitudeFraction1/10 + altitudeFull);
    return iComma;
}


//****** Height of geoid (mean sea level) above WGS84 llipsoid****
uint8_t parseHeightOfGeoid(uint8_t * buffer, NMEA_Altitude_s *data, uint8_t iComma)
{
    //$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\n
    uint8_t geoidFraction1;
    uint16_t geoidFull = 0;
    int8_t signum = 1;
    if(buffer[iComma+1]=='-')
    {
        signum = -1;
        iComma++;
    }
    for(iComma++; buffer[iComma] != '.'; iComma++)
    {
        geoidFull = geoidFull * 10 + (buffer[iComma] - '0');
    }
    iComma++;
    geoidFraction1 = buffer[iComma] - '0';
    iComma++;
    data->geoidHeight = signum * ((float)geoidFraction1/10 + geoidFull);

    data->altitudeUIntWithGeoid =  data->altitudeUInt - geoidFull;
    return iComma;
}



NMEA_System_e getGnssSystemInNmea(uint8_t *buffer)
{
	if(buffer[1] == 'G' && buffer[2] == 'N')
	{
		return NMEA_System_Main;
	}
	else if(buffer[1] == 'G' && buffer[2] == 'P')
	{
		return NMEA_System_GPS;
	}
	else if(buffer[1] == 'G' && buffer[2] == 'L')
	{
		return NMEA_System_Glonass;
	}
	else
	{
		return NMEA_System_Unknown;
	}
}

NMEA_Message_e getTypeInNmea(uint8_t *buffer)
{
	if((buffer[3] == 'G')&&(buffer[4] == 'G')&&(buffer[5] == 'A'))//GP GGA
	{
		return NMEA_Message_GGA;
	}
	else if((buffer[3] == 'G')&&(buffer[4] == 'S')&&(buffer[5] == 'A'))
	{
		return NMEA_Message_GSA;
	}
	else if((buffer[3] == 'R')&&(buffer[4] == 'M')&&(buffer[5] == 'C'))//GP GGA
	{
		return NMEA_Message_RMC;
	}
	else if((buffer[3] == 'G')&&(buffer[4] == 'L')&&(buffer[5] == 'L'))//GP GGA
	{
		return NMEA_Message_GLL;
	}
	else if((buffer[3] == 'G')&&(buffer[4] == 'S')&&(buffer[5] == 'V'))//GP GGA
	{
		return NMEA_Message_GSV;
	}
	else if((buffer[3] == 'V')&&(buffer[4] == 'T')&&(buffer[5] == 'G'))//GP GGA
	{
		return NMEA_Message_VTG;
	}
	else if((buffer[3] == 'G')&&(buffer[4] == 'R')&&(buffer[5] == 'S'))//GP GGA
	{
		return NMEA_Message_GRS;
	}
	else if((buffer[3] == 'T')&&(buffer[4] == 'X')&&(buffer[5] == 'T'))//GP GGA
	{
		return NMEA_Message_TXT;
	}
	else
	{
		return NMEA_Message_Unknown;
	}
}

NMEA_Params_s checkMessage_NMEA(uint8_t *buffer)
{
	NMEA_Params_s params;

	if(checkChecksum(buffer))
	{
		params.message = NMEA_Message_Error;
		params.system = NMEA_System_Error;
		return params;
	}

	params.message = getTypeInNmea(buffer);
	params.system = getGnssSystemInNmea(buffer);
	return params;
}

////checksum and check if GGA or GSA
//void proceedNMEABuffer(uint8_t *buffer, uint8_t *data)
//{
//
//    if(checkChecksum(buffer))
//    {
//        return;
//    }
//
//    if((buffer[3] == 'G')&&(buffer[4] == 'G')&&(buffer[5] == 'A'))//GP GGA
//    {
//    	proceedGGAparsing(buffer);
//    }
////    {
////        proceedGGAparsing(buffer);
////    }
////    else if((buffer[3] == 'G')&&(buffer[4] == 'S')&&(buffer[5] == 'A'))//GP GSA
////    {
////        proceedGSAparsing(buffer);
////    }
////    else if((buffer[3] == 'G')&&(buffer[4] == 'S')&&(buffer[5] == 'V'))//GP GSA
////    {
////        proceedGSVparsing(buffer);
////    }
////    else if((buffer[3] == 'G')&&(buffer[4] == 'L')&&(buffer[5] == 'L'))//GP GSA
////    {
////       // proceedGLLparsing(buffer);//redundant
////    }
////    else if((buffer[3] == 'R')&&(buffer[4] == 'M')&&(buffer[5] == 'C'))//GP GSA
////    {
////        proceedRMCparsing(buffer);
////    }
////    else if((buffer[3] == 'V')&&(buffer[4] == 'T')&&(buffer[5] == 'G'))//GP GSA
////    {
////       // proceedVTGparsing(buffer);//redundant
////    }
////    else if((buffer[3] == 'Z')&&(buffer[4] == 'D')&&(buffer[5] == 'A'))//GP GSA
////    {
////        // proceedZDAparsing(buffer);//redundant
////    }
//}

int8_t checkChecksum(uint8_t * buffer)
{
	uint8_t crc_rec = 0;
	uint8_t crc_calc = 0;
	uint8_t charToNum;
	uint8_t i;

	for(i = 1; i < MAXIMUM_GPS_BUFFER_LENGTH; i++)
	{
		if(buffer[i] == '*') break;
		crc_calc ^= buffer[i];
	}

	charToNum = buffer[++i] - 48;
	crc_rec = ((charToNum / 17 * 10) + charToNum % 17) << 4;
	charToNum = buffer[++i] - 48;
	crc_rec |= ((charToNum / 17 * 10) + charToNum % 17);

	if(crc_calc == crc_rec) return 0;
	else return 1;
}


#endif /* NMEA_C_ */
